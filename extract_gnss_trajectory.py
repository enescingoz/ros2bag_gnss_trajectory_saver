import os
import argparse
import shutil
import utm
import pylas
import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import get_typestore, Stores
from sensor_msgs.msg import NavSatFix
import csv


def navsatfix_to_utm(lat, lon, alt):
    u, v, zone_number, zone_letter = utm.from_latlon(lat, lon)
    return u, v, alt, zone_number, zone_letter


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


def save_trajectory_as_csv(trajectory, filename):
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['x', 'y', 'z'])
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
    print("Saving trajectories...")

    local_trajectory, utm_trajectory, lon_lat_alt, first_utm, utm_zone = [], [], [], None, None
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

                if first_utm is None:
                    first_utm = utm_coords[:3]
                local_trajectory.append(tuple(utm_coords[i] - first_utm[i] for i in range(3)))

    transform_matrix = euler_to_matrix(179.275, -0.007, 179.690, 0.0, 0.0, -0.151)
    
    local_las_filename = os.path.join(output_dir, 'local_gnss_trajectory.las')
    save_trajectory_as_las(local_trajectory, local_las_filename)
    print(f'Saved: {local_las_filename}')

    utm_las_filename = os.path.join(output_dir, 'utm_trajectory.las')
    save_trajectory_as_las(utm_trajectory, utm_las_filename)
    print(f'Saved: {utm_las_filename}')

    local_kitti_filename = os.path.join(output_dir, 'local_kitti_trajectory.txt')
    save_kitti_format(local_trajectory, local_kitti_filename, transform_matrix)
    print(f'Saved: {local_kitti_filename}')

    utm_kitti_filename = os.path.join(output_dir, 'utm_kitti_trajectory.txt')
    save_kitti_format(utm_trajectory, utm_kitti_filename)
    print(f'Saved: {utm_kitti_filename}')

    local_csv_filename = os.path.join(output_dir, 'local_gnss_trajectory.csv')
    save_trajectory_as_csv(local_trajectory, local_csv_filename)

    utm_csv_filename = os.path.join(output_dir, 'utm_trajectory.csv')
    save_trajectory_as_csv(utm_trajectory, utm_csv_filename)

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
