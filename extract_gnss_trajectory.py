import os
import argparse
import shutil
import utm
import pylas
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

def main():
    parser = argparse.ArgumentParser(description='Extract and save GNSS trajectory from a ROS 2 bag file.')
    parser.add_argument('--bag', type=str, required=True, help='Path to the ROS 2 bag file folder which contains .db3 and metadata files.')
    parser.add_argument('--topic', type=str, required=True, help='GNSS topic name which message type is sensor_msgs/NavSatFix.')
    args = parser.parse_args()

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
    with Reader(args.bag) as reader:
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == args.topic:
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
    
    # Save the trajectories as LAS files
    local_las_filename = os.path.join(output_dir, 'local_gnss_trajectory.las')
    save_trajectory_as_las(local_trajectory, local_las_filename)
    print(f'Saved: {local_las_filename}')

    utm_las_filename = os.path.join(output_dir, 'utm_trajectory.las')
    save_trajectory_as_las(utm_trajectory, utm_las_filename)
    print(f'Saved: {utm_las_filename}')

    # Print UTM zone
    print(f"UTM Zone: {utm_zone}")

if __name__ == '__main__':
    main()
