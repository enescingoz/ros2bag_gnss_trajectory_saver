
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
pip3 install rosbags utm pylas
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
- `/gnss_topic_name` should be replaced with the actual topic name that contains the GNSS data you want to extract.

### Script Description

1. The script reads a ROS 2 bag file specified by the `--bag` argument.
2. It extracts GNSS messages from the topic specified by the `--topic` argument.
3. Each GNSS message is converted to UTM coordinates.
4. Local coordinates relative to the first UTM coordinate are calculated.
5. The local GNSS trajectory and UTM trajectory are saved to `.las` files in the `output` directory.
6. The output directory is cleaned before saving new trajectories.
7. The UTM zone is printed at the end of the script.

### Output Files

- Local GNSS trajectory file: `output/local_gnss_trajectory.las`
- UTM trajectory file: `output/utm_trajectory.las`

## Example

```sh
python3 extract_gnss_trajectory.py --bag /path/to/rosbag/folder --topic /gnss_topic_name
```

This command will create the `output` directory, clean it, and save the local and UTM GNSS trajectories in it.

## TODO

- Save MGRS trajectory
