# Usage

The commands below assume that wavemap's workspace is built and sourced, as described on the [installation page](installation.md).

## Configuration

All of the example configs from the ROS1 package are installed unmodified, in `$(ros2 pkg prefix wavemap2_ros)/share/wavemap2_ros/config` (source: [`wavemap2_ros/config`](../wavemap2_ros/config)).
They cover Ouster OS0 and OS1 LiDARs, the Livox MID-360, PMD Pico depth cameras, the Azure Kinect and the Panoptic Mapping dataset.
Your own ROS1 configs can be used as they are, and all settings are explained on the upstream [parameters page](https://ethz-asl.github.io/wavemap/pages/parameters/index.html).

Unlike in ROS1, the config file must always be passed explicitly with `param_file`.
The settings you will usually need to change first are the input's `topic_name`, which must match your sensor driver's topic, and `world_frame`, which must match your odometry's fixed frame (see [odometry](odometry.md)).

## Launch files

The [launch files](../wavemap2_ros/launch) mirror ROS1's and are written in ROS2's XML launch format:

| Launch file | Purpose |
|---|---|
| `wavemap_server.launch.xml` | Runs the server on live topics, optionally replaying rosbags in real time |
| `rosbag_processor.launch.xml` | Processes rosbags as fast as possible |
| `datasets/panoptic_mapping/panoptic_mapping_rgbd_flat.launch.xml` | Panoptic Mapping flat dataset demo |
| `datasets/newer_college/newer_college_os0_{cloister,math,mine,park}.launch.xml` | Newer College dataset demos |

The two general launch files take these arguments:

| Argument | Default | Meaning |
|---|---|---|
| `param_file` | (none, required) | Wavemap config file |
| `show_rviz` | `true` | Start Rviz2 with wavemap's map display |
| `rviz_config` | `wavemap2_ros`'s `config/rviz/default.rviz` | Rviz2 config to load |
| `debug_mode` | `false` | Run wavemap in `gdb` |
| `rosbag_path` | empty | Rosbags to play or process, separated by spaces |
| `rosbag_skip_first_n_sec` | `0` | Server only: skip the start of the replayed rosbags |
| `exit_when_done` | `true` | Rosbag processor only: shut down the launch once wavemap exits |

As in ROS1, relative rosbag paths are resolved with respect to `ROS_HOME` (default `~/.ros`).
Unlike in ROS1, `use_sim_time` is passed to each node, since ROS2 has no global parameters.

### Server

To run the wavemap server on live data, use:

```bash
ros2 launch wavemap2_ros wavemap_server.launch.xml param_file:=/path/to/your_config.yaml
```

To replay rosbags in real time instead, add `rosbag_path:="/path/to/first_bag /path/to/second_bag"`.
The launch file then plays them with `ros2 bag play --clock` and runs wavemap and Rviz2 on simulated time.

### Rosbag processor

To process one or more rosbags as fast as possible, use:

```bash
ros2 launch wavemap2_ros rosbag_processor.launch.xml param_file:=/path/to/your_config.yaml rosbag_path:="/path/to/first_bag /path/to/second_bag"
```

The rosbag processor reads the bags directly rather than subscribing to topics, so it is deterministic and does not depend on the [DDS settings](dds.md).
It takes the sensor poses from a live `TransformListener`, so transforms that are not in the bags, such as static sensor extrinsics, must be published while it runs.
As in ROS1, it stays alive once it is done, so the map can be inspected in Rviz2, unless `show_rviz:=false` is set, in which case it exits.

## Live inputs

Wavemap subscribes to its depth image and pointcloud inputs with ROS2's sensor data QoS profile (best effort), so it receives data from both best-effort and reliable publishers.

When large pointclouds, such as an Ouster's, arrive over live topics, the DDS middleware needs suitable settings, or it silently loses most of them.
The [DDS page](dds.md) describes them for Fast DDS and Cyclone DDS.

## Demos

The datasets and their download links are described on the upstream [demos page](https://ethz-asl.github.io/wavemap/pages/demos.html).
The ROS2 launch files look for them in the same default locations as the ROS1 ones, relative to `ROS_HOME`, i.e. under `~/data` for a native install and under the mounted `/home/ci/data` in the [Docker container](installation.md#running-a-container).
With the data in these locations, the path arguments below can be left out.

### Panoptic Mapping flat dataset

Download and extract the [flat dataset](https://doi.org/10.3929/ethz-c-000788335), and run:

```bash
ros2 launch wavemap2_ros panoptic_mapping_rgbd_flat.launch.xml base_path:=/path/to/flat_dataset/run1
```

The default `base_path` is `~/data/panoptic_mapping/flat_dataset/run1`.

### Newer College dataset

The Cloister, Math, Mine and Park sequences each need the dataset's sensor rosbags and a rosbag with pre-recorded FAST-LIO odometry, such as `nc_cloister_fast_lio_odom.bag`, both linked from the upstream demos page.
Unlike in ROS1, the odometry rosbag is required, since the ROS2 launch files cannot run FAST-LIO live.
These are ROS1 rosbags, so convert them to ROS2 first, for example with the [rosbags](https://ternaris.gitlab.io/rosbags/) Python package, keeping their names without the `.bag` extension, and run e.g.:

```bash
ros2 launch wavemap2_ros newer_college_os0_cloister.launch.xml rosbag_dir:=/path/to/converted/cloister
```

The default `rosbag_dir` for the Cloister sequence is `~/data/newer_college/2021-ouster-os0-128-alphasense/collection2/cloister`, and `only_first_rosbag:=true` processes only the sequence's first rosbag.

By default these process the bags in batch mode with the rosbag processor, using the odometry bag for the LiDAR poses.
To replay them in real time with the server instead, add `use_batch_mode:=false`.

## Visualizing maps in Rviz2

The launch files open Rviz2 with wavemap's map display already configured to show the server's map, on the `/wavemap/map` topic.
On machines without a display, such as a server or a Docker container without X forwarding, pass `show_rviz:=false`.
To add the display to Rviz2 yourself, click `Add`, select `wavemap2_rviz_plugin/WavemapMap`, and set its topic.

Saved maps can also be viewed without running the server:

1. Run `rviz2` and add the `wavemap2_rviz_plugin/WavemapMap` display.
1. Under `Source`, select `File`.
1. Load the map by clicking `Loaded map` → `Choose file`.

The map file format is the same as in ROS1, so maps saved with ROS1, such as the pre-built maps linked from the upstream demos page, can be opened as they are.

The display's `Render voxels` and `Render slice` options control how the map is drawn.
Its `Full map update` button asks the server to republish its whole map, and its `Reset server` button asks the server to reset its map, which the server only allows if its config enables `allow_reset_map_service`.

Rviz2's wavemap plugin must be built with the same [map message layout](map_message_layout.md) as the server, or it shows no map.

## Services

The server offers the same services as in ROS1:

```bash
ros2 service call /wavemap/save_map wavemap2_msgs/srv/FilePath "{file_path: /path/to/map.wvmp}"
ros2 service call /wavemap/load_map wavemap2_msgs/srv/FilePath "{file_path: /path/to/map.wvmp}"
ros2 service call /wavemap/reset_map std_srvs/srv/Trigger
ros2 service call /wavemap/map_request_full std_srvs/srv/Empty
```

Saved maps can subsequently be used [in C++](https://ethz-asl.github.io/wavemap/pages/tutorials/cpp.html#cpp-code-examples-read-map) (with or without ROS) and [in Python](https://ethz-asl.github.io/wavemap/pages/tutorials/python.html#python-code-examples-read-map).

## Publishing maps

Wavemap's ROS2 server offers the same ways to publish its maps to ROS topics as the ROS1 server, with the same config options.
Please refer to the upstream documentation on [ROS1 map operations](https://ethz-asl.github.io/wavemap/pages/parameters/map_operations.html#configuration-map-operations-ros1) for an overview of the available options.

The map is published with reliable QoS.
To subscribe to it from your own node, both reliable and best-effort subscriptions work.
Your node must be built with the same [map message layout](map_message_layout.md) as the server.

## Your own data

The only requirements to run wavemap on your own dataset or robot are:

1. a source of depth measurements,
1. sensor pose (estimates) for each measurement.

We usually use depth measurements from depth cameras or 3D LiDARs, but any source would work as long as a sufficiently good [projection](https://ethz-asl.github.io/wavemap/pages/parameters/measurement_integrators.html#configuration-projection-models) and [measurement](https://ethz-asl.github.io/wavemap/pages/parameters/measurement_integrators.html#configuration-measurement-models) model is available.
Wavemap's ROS2 interface natively supports depth image and pointcloud inputs, and automatically queries the sensor poses from the TF tree.
The [odometry page](odometry.md) describes how to provide the poses.

The ROS2 interface does not yet include the launch files for the online sensor setups, which in ROS1 also start the sensor drivers and FAST-LIO.
Instead, start your sensor driver and odometry separately, publish the static transforms between your sensor and odometry frames, and run `wavemap_server.launch.xml` with your config.
