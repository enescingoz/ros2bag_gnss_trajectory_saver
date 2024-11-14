
# ROS2 Bag to Local and UTM GNSS Trajectories Converter

This script extracts GNSS data from a ROS 2 bag file, converts the data to UTM coordinates, calculates local coordinates relative to the first UTM coordinate, and saves the trajectories to `.las` files.

## Dependencies

To run this script, you need to have the following dependencies installed:

- Python 3.8 or higher
- `rosbags`
- `utm`
- `pylas`
- `sensor_msgs`

You can install these dependencies using `pip3`:

```sh
python3 -m venv bag2gnsstrajectory
source bag2gnsstrajectory/bin/activate

pip3 install rosbags utm pylas geographiclib
```

## Usage

### Command Line Arguments

- `--bag`: Path to the ROS 2 bag file folder which contains `.db3` and metadata files.
- `--topic`: GNSS topic name which message type is `sensor_msgs/NavSatFix`.

### Example

```sh
python3 extract_gnss_trajectory.py --bag /path/to/rosbag/folder --topic /gnss_topic_name
```

In this example:
- `/path/to/rosbag/folder` should be replaced with the actual path to your ROS 2 bag file folder.
- `/gnss_topic_name` should be replaced with the actual topic name that contains the GNSS data you want to extract. (sensor_msgs/msg/NavSatFix)

### Script Description
1. The script reads a ROS 2 bag file specified by the --bag argument.
2. It extracts GNSS messages from the topic specified by the --topic argument.
3. Each GNSS message is converted to UTM coordinates.
4. Local coordinates relative to the first UTM coordinate are calculated.
5. The GNSS data is saved in multiple formats for convenience and compatibility.

### Output Files

All output files are saved in an output directory. The directory is cleaned before saving new trajectories.

##### 1. Local GNSS Trajectory
- LAS File: output/local_gnss_trajectory.las
- KITTI Format: output/local_kitti_trajectory.txt
- CSV File: output/local_gnss_trajectory.csv

##### 2. UTM GNSS Trajectory
- LAS File: output/utm_trajectory.las
- KITTI Format: output/utm_kitti_trajectory.txt
- CSV File: output/utm_trajectory.csv
- CSV File: output/utm_trajectory_yaw.csv (UTM trajectory with yaw angles)

##### 3. MGRS GNSS Trajectory
- LAS File: output/mgrs_trajectory.las
- CSV File: output/mgrs_trajectory.csv
- CSV File: output/mgrs_trajectory_yaw.csv (MGRS trajectory with yaw angles)

##### 4. Latitude, Longitude, and Altitude
- TXT File: output/lat_lon_alt_trajectory.txt
- CSV File: output/lat_lon_alt_trajectory.csv

Each of these formats provides the trajectory in a different structure for use in analysis or visualization applications.

## Example

```sh
python3 extract_gnss_trajectory.py --bag /path/to/rosbag/folder --topic /gnss_topic_name
```

This command will create the `output` directory, clean it, and save the local and UTM GNSS trajectories in it.
