import os
import argparse
import shutil
import utm
import csv
import pylas
import numpy as np
import math
from rosbags.rosbag2 import Reader
from rosbags.typesys import get_typestore, Stores
from sensor_msgs.msg import NavSatFix



def navsatfix_to_utm(lat, lon, alt):
    u, v, zone_number, zone_letter = utm.from_latlon(lat, lon)
    is_north = zone_letter >= 'N'
    return u, v, alt, zone_number, zone_letter, is_north


def utm_to_mgrs(utm_coords):
    u, v, alt, zone_number, zone_letter, is_north = utm_coords
    
    mgrs_x = u % 1e5
    mgrs_y = v % 1e5

    return mgrs_x, mgrs_y, alt


def calculate_yaw_trajectory(trajectory):
    # Initialize an empty list to store the new trajectory with yaw angles
    trajectory_with_yaw = []
    
    # Return empty list if trajectory has fewer than two points
    if len(trajectory) < 2:
        return trajectory_with_yaw
    
    # Hold the first coordinate as the starting point
    held_x, held_y, held_z = trajectory[0]
    
    # Iterate over each point in the trajectory starting from the second point
    for i in range(1, len(trajectory)):
        x, y, z = trajectory[i]
        
        # Calculate the yaw angle between held point and current point
        delta_x = x - held_x
        delta_y = y - held_y
        yaw = math.atan2(delta_y, delta_x)  # Angle in radians
        
        # Calculate the Euclidean distance between the held point and current point
        distance = math.sqrt(delta_x ** 2 + delta_y ** 2 + (z - held_z) ** 2)
        
        # Only append if the distance is greater than 4 meters
        if distance > 4:
            trajectory_with_yaw.append((held_x, held_y, held_z, yaw))
            
            # Update the held coordinate to the current point
            held_x, held_y, held_z = x, y, z
    
    # Add the final point to ensure it’s included
    last_x, last_y, last_z = trajectory[-1]
    final_yaw = trajectory_with_yaw[-1][3] if trajectory_with_yaw else 0
    trajectory_with_yaw.append((last_x, last_y, last_z, final_yaw))
    
    return trajectory_with_yaw


def euler_to_matrix(yaw, pitch, roll, x, y, z):
    yaw, pitch, roll = np.radians([yaw, pitch, roll])

    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                   [np.sin(yaw), np.cos(yaw), 0],
                   [0, 0, 1]])

    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                   [0, 1, 0],
                   [-np.sin(pitch), 0, np.cos(pitch)]])

    Rx = np.array([[1, 0, 0],
                   [0, np.cos(roll), -np.sin(roll)],
                   [0, np.sin(roll), np.cos(roll)]])

    R = Rz @ Ry @ Rx
    T = np.identity(4)
    T[:3, :3], T[:3, 3] = R, [x, y, z]

    return T


def save_trajectory_as_las(trajectory, filename):
    las = pylas.create()
    points = [(coord[0], coord[1], coord[2]) for coord in trajectory]
    las.x, las.y, las.z = zip(*points)
    las.write(filename)
    print(f'Saved: {filename}')


def save_kitti_format(trajectory, filename, transform_matrix=None):
    with open(filename, 'w') as f:
        for coord in trajectory:
            if transform_matrix is not None:
                point = np.array([*coord, 1]).reshape(4, 1)
                tx, ty, tz = (transform_matrix @ point).flatten()[:3]
            else:
                tx, ty, tz = coord

            pose = [
                1, 0, 0, tx,
                0, 1, 0, ty,
                0, 0, 1, tz
            ]
            pose_str = ' '.join(map(str, pose))
            f.write(pose_str + '\n')

    print(f'Saved: {filename}')


def save_trajectory_as_csv(trajectory, filename):
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['x', 'y', 'z'])
        writer.writerows(trajectory)
    print(f'Saved: {filename}')


def save_trajectory_with_yaw_as_csv(trajectory, filename):
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['x', 'y', 'z', 'yaw'])
        writer.writerows(trajectory)
    print(f'Saved: {filename}')


def save_lon_lat_alt_as_txt(lon_lat_alt, filename):
    with open(filename, 'w') as f:
        for lon, lat, alt in lon_lat_alt:
            f.write(f"{lon} {lat} {alt}\n")
    print(f'Saved: {filename}')


def save_lon_lat_alt_as_csv(lon_lat_alt, filename):
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['longitude', 'latitude', 'altitude'])
        writer.writerows(lon_lat_alt)
    print(f'Saved: {filename}')


def extract_gnss(bag_file, topic_name):
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    print("Extracting GNSS data...")

    local_trajectory, utm_trajectory, mgrs_trajectory, lon_lat_alt, first_utm, utm_zone = [], [], [], [], None, None
    output_dir = 'output'
    os.makedirs(output_dir, exist_ok=True)

    with Reader(bag_file) as reader:
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == topic_name:
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                lon_lat_alt.append((msg.longitude, msg.latitude, msg.altitude))

                utm_coords = navsatfix_to_utm(msg.latitude, msg.longitude, msg.altitude)
                utm_trajectory.append(utm_coords[:3])
                utm_zone = utm_coords[3:]

                mgrs_trajectory.append(utm_to_mgrs(utm_coords))

                if first_utm is None:
                    first_utm = utm_coords[:3]
                local_trajectory.append(tuple(utm_coords[i] - first_utm[i] for i in range(3)))


    utm_trajectory_with_yaw = calculate_yaw_trajectory(utm_trajectory)
    mgrs_trajectory_with_yaw = calculate_yaw_trajectory(mgrs_trajectory)

    transform_matrix = euler_to_matrix(179.275, -0.007, 179.690, 0.0, 0.0, -0.151)
    

    print("Saving trajectories...")
    ### SAVE LOCAL TRAJECTORY
    local_las_filename = os.path.join(output_dir, 'local_gnss_trajectory.las')
    save_trajectory_as_las(local_trajectory, local_las_filename)

    local_kitti_filename = os.path.join(output_dir, 'local_kitti_trajectory.txt')
    save_kitti_format(local_trajectory, local_kitti_filename, transform_matrix)

    local_csv_filename = os.path.join(output_dir, 'local_gnss_trajectory.csv')
    save_trajectory_as_csv(local_trajectory, local_csv_filename)


    ### SAVE UTM TRAJECTORY
    utm_las_filename = os.path.join(output_dir, 'utm_trajectory.las')
    save_trajectory_as_las(utm_trajectory, utm_las_filename)

    utm_kitti_filename = os.path.join(output_dir, 'utm_kitti_trajectory.txt')
    save_kitti_format(utm_trajectory, utm_kitti_filename)

    utm_csv_filename = os.path.join(output_dir, 'utm_trajectory.csv')
    save_trajectory_as_csv(utm_trajectory, utm_csv_filename)

    utm_yaw_csv_filename = os.path.join(output_dir, 'utm_trajectory_yaw.csv')
    save_trajectory_with_yaw_as_csv(utm_trajectory_with_yaw, utm_yaw_csv_filename)


    ### SAVE MGRS TRAJECTORY
    mgrs_las_filename = os.path.join(output_dir, 'mgrs_trajectory_yaw.las')
    save_trajectory_as_las(mgrs_trajectory_with_yaw, mgrs_las_filename)

    mgrs_csv_filename = os.path.join(output_dir, 'mgrs_trajectory.csv')
    save_trajectory_as_csv(mgrs_trajectory, mgrs_csv_filename)

    mgrs_yaw_csv_filename = os.path.join(output_dir, 'mgrs_trajectory_yaw.csv')
    save_trajectory_with_yaw_as_csv(mgrs_trajectory_with_yaw, mgrs_yaw_csv_filename)


    ### SAVE LAT-LON TRAJECTORY
    lon_lat_alt_txt_filename = os.path.join(output_dir, 'longitude_latitude_altitude.txt')
    save_lon_lat_alt_as_txt(lon_lat_alt, lon_lat_alt_txt_filename)

    lon_lat_alt_csv_filename = os.path.join(output_dir, 'longitude_latitude_altitude.csv')
    save_lon_lat_alt_as_csv(lon_lat_alt, lon_lat_alt_csv_filename)

    print(f"UTM Zone: {utm_zone}")


def main():
    parser = argparse.ArgumentParser(description='Extract and save GNSS trajectory from a ROS 2 bag file.')
    parser.add_argument('--bag', type=str, required=True, help='Path to the ROS 2 bag file folder.')
    parser.add_argument('--topic', type=str, required=True, help='GNSS topic name.')
    args = parser.parse_args()
    extract_gnss(args.bag, args.topic)


if __name__ == '__main__':
    main()
