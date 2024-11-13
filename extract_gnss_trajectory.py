import os
import argparse
import shutil
import utm
import pylas
import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import get_typestore, Stores
from sensor_msgs.msg import NavSatFix


def navsatfix_to_utm(lat, lon, alt):
    u, v, zone_number, zone_letter = utm.from_latlon(lat, lon)
    return u, v, alt, zone_number, zone_letter


def save_trajectory_as_las(trajectory, filename):
    # Create a LAS file with the trajectory data
    las = pylas.create()
    points = []
    for coord in trajectory:
        points.append((coord[0], coord[1], coord[2]))

    las.x, las.y, las.z = zip(*points)
    
    # Save the LAS file
    las.write(filename)


def euler_to_matrix(yaw, pitch, roll, x, y, z):
    # Convert angles from degrees to radians
    yaw = np.radians(yaw)
    pitch = np.radians(pitch)
    roll = np.radians(roll)

    # Rotation matrices around the yaw, pitch, and roll axes
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    # Combined rotation matrix
    R = np.dot(Rz, np.dot(Ry, Rx))

    # Transformation matrix
    T = np.identity(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]

    return T


def save_kitti_format(trajectory, filename, transform_matrix=None):
    # Save each pose in KITTI format
    with open(filename, 'w') as f:
        for coord in trajectory:
            if transform_matrix is not None:
                # Transform the point using the matrix
                point = np.array([coord[0], coord[1], coord[2], 1]).reshape(4, 1)
                transformed_point = np.dot(transform_matrix, point).flatten()
                tx, ty, tz = transformed_point[:3]
            else:
                tx, ty, tz = coord

            # Identity rotation matrix with translation
            pose = [
                1, 0, 0, tx,
                0, 1, 0, ty,
                0, 0, 1, tz
            ]
            # Convert to string and write to file
            pose_str = ' '.join(map(str, pose))
            f.write(pose_str + '\n')

def extract_gnss(bag_file, topic_name):
    typestore = get_typestore(Stores.ROS2_HUMBLE)  # Update this to your specific ROS 2 distribution

    # Print message before processing
    print("Saving trajectories...")

    # Initialize variables
    local_trajectory = []
    utm_trajectory = []
    first_utm = None

    # Variable to store UTM zone
    utm_zone = None

    # Clean the output directory
    output_dir = 'output'
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    os.makedirs(output_dir)

    # Open the ROS 2 bag file
    with Reader(bag_file) as reader:
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == topic_name:
                # Deserialize the NavSatFix message
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                
                # Convert to UTM coordinates
                utm_coords = navsatfix_to_utm(msg.latitude, msg.longitude, msg.altitude)
                utm_trajectory.append(utm_coords[:3])
                utm_zone = utm_coords[3:]

                # Store the first UTM coordinate
                if first_utm is None:
                    first_utm = utm_coords[:3]

                # Calculate local coordinates relative to the first UTM coordinate
                local_coords = (
                    utm_coords[0] - first_utm[0],
                    utm_coords[1] - first_utm[1],
                    utm_coords[2] - first_utm[2]
                )

                # Add the local coordinates to the trajectory
                local_trajectory.append(local_coords)


    # Define transformation matrix
    transform_matrix = euler_to_matrix(179.275, -0.007, 179.690, 0.0, 0.0, -0.151)
    
    # Save the trajectories as LAS files
    local_las_filename = os.path.join(output_dir, 'local_gnss_trajectory.las')
    save_trajectory_as_las(local_trajectory, local_las_filename)
    print(f'Saved: {local_las_filename}')

    utm_las_filename = os.path.join(output_dir, 'utm_trajectory.las')
    save_trajectory_as_las(utm_trajectory, utm_las_filename)
    print(f'Saved: {utm_las_filename}')

    # Save the local trajectories in KITTI format (transformed)
    local_kitti_filename = os.path.join(output_dir, 'local_kitti_trajectory.txt')
    save_kitti_format(local_trajectory, local_kitti_filename, transform_matrix)
    print(f'Saved: {local_kitti_filename}')

    # Save the UTM trajectories in KITTI format (without transformation)
    utm_kitti_filename = os.path.join(output_dir, 'utm_kitti_trajectory.txt')
    save_kitti_format(utm_trajectory, utm_kitti_filename)
    print(f'Saved: {utm_kitti_filename}')

    # Print UTM zone
    print(f"UTM Zone: {utm_zone}")


def main():
    parser = argparse.ArgumentParser(description='Extract and save GNSS trajectory from a ROS 2 bag file.')
    parser.add_argument('--bag', type=str, required=True, help='Path to the ROS 2 bag file folder which contains .db3 and metadata files.')
    parser.add_argument('--topic', type=str, required=True, help='GNSS topic name which message type is sensor_msgs/NavSatFix.')
    args = parser.parse_args()


    extract_gnss(args.bag, args.topic)

    

if __name__ == '__main__':
    main()
