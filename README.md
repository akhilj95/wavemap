# Wavemap for ROS2

This repository is a fork of [wavemap](https://github.com/ethz-asl/wavemap), developed by the Autonomous Systems Lab at ETH Zurich, that adds a ROS2 interface.
The interface targets [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/) on Ubuntu 24.04 (Noble).
It provides the same server and rosbag processor as wavemap's ROS1 interface, with the same topic and service names, and reads the same YAML config files unmodified.

> **Map messages differ from ROS1 by default.**
> The octree nodes in wavemap's ROS2 map messages are sent as flat arrays (the `flat` layout), because DDS sends these much faster than ROS1's array of `WaveletOctreeNode` messages.
> ROS1's layout is available as the `nested` build option, for example to bridge maps to ROS1.
> Every node that exchanges maps with wavemap, including Rviz2 with wavemap's plugin, must be built with the same layout as the server, or it cannot read the maps.
> Saved `.wvmp` map files do not depend on the layout, and are the same as in ROS1.
> See [map message layout](interfaces/ros2/docs/map_message_layout.md).

The ROS2 interface lives in [`interfaces/ros2`](interfaces/ros2).
Wavemap's library, its C++ and Python APIs and its ROS1 interface are unchanged from upstream.

## About wavemap

Wavemap is a hierarchical, multi-resolution volumetric mapping framework.
It achieves memory and computational efficiency by combining Haar wavelet compression and a coarse-to-fine measurement integration scheme. The framework is very flexible and supports several data structures, measurement integration methods, and sensor models out of the box.
Its ROS interfaces can, for example, easily be configured to fuse multiple sensor inputs, such as a LiDAR configured with a range of 20m and several depth cameras up to a resolution of 1cm, into a single multi-resolution occupancy grid map.

Wavemap provides [C++](https://ethz-asl.github.io/wavemap/pages/tutorials/cpp.html) and [Python](https://ethz-asl.github.io/wavemap/pages/tutorials/python.html) APIs and an interface to [ROS1](https://ethz-asl.github.io/wavemap/pages/tutorials/ros1.html), all documented on [wavemap's documentation site](https://ethz-asl.github.io/wavemap/).

## Building

Both builds by default use the `flat` map message layout (which differs from the ROS1 structure). The ROS1 message structure can be selected with the `nested` option.
More details, including the VS Code devcontainer, are on the [installation page](interfaces/ros2/docs/installation.md).

### Docker

```bash
git clone https://github.com/akhilj95/wavemap.git
cd wavemap
docker build --target=workspace-built --tag=wavemap2 -f tooling/docker/ros2/full.Dockerfile .
```

For the `nested` layout, add `--build-arg WAVEMAP2_MSGS_LAYOUT=nested`.

To start a container that can open Rviz2 on your display, with your `~/data` folder mounted at `/home/ci/data`, run:

```bash
xhost +local:docker
docker run --rm -it --network host \
  -e DISPLAY="${DISPLAY}" -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ~/data:/home/ci/data \
  wavemap2
```

The workspace in the container is already built and sourced.
The image uses Cyclone DDS, already configured for wavemap's large messages, rather than ROS 2 Jazzy's default, Fast DDS. Nodes outside the container that communicate with wavemap must use it too, see the [DDS page](interfaces/ros2/docs/dds.md#cyclone-dds).

### Native

With [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) installed (`ros-jazzy-ros-base` is enough):

```bash
sudo apt install git build-essential python3-rosdep python3-colcon-common-extensions
git clone https://github.com/akhilj95/wavemap.git ~/wavemap

# Make a ros2 workspace and link the ROS2 packages into it instead of the whole repo
mkdir -p ~/ros2_ws/src
ln -s ~/wavemap/interfaces/ros2 ~/ros2_ws/src/wavemap

source /opt/ros/jazzy/setup.bash
sudo rosdep init  # Skip if rosdep is already initialized
rosdep update
rosdep install -y --from-paths ~/ros2_ws/src --ignore-src
cd ~/ros2_ws
colcon build --packages-up-to wavemap2_all --cmake-args -DCMAKE_BUILD_TYPE=Release
source ~/ros2_ws/install/setup.bash
```

Only `interfaces/ros2` is linked into the workspace, since colcon would otherwise treat the whole repository as a single package.
For the `nested` layout, add `-DWAVEMAP2_MSGS_LAYOUT=nested` after `-DCMAKE_BUILD_TYPE=Release`.

## Running the server

Wavemap needs two inputs: depth measurements, as pointclouds or depth images, and the sensor's pose for each measurement, which it looks up in TF.
**The poses have to come from an odometry system that publishes them to TF.**
Wavemap does not estimate them itself.
The [odometry page](interfaces/ros2/docs/odometry.md) describes the TF frames wavemap needs and how to connect it to iG-LIO.

Copy one of the example configs, set its input topic to your sensor's (`topic_name`) and its `world_frame` to your odometry's fixed frame, and start the server with:

```bash
cp $(ros2 pkg prefix wavemap2_ros)/share/wavemap2_ros/config/wavemap_ouster_os0.yaml ~/my_config.yaml
ros2 launch wavemap2_ros wavemap_server.launch.xml param_file:=$HOME/my_config.yaml
```

The example configs cover Ouster and Livox LiDARs and several depth cameras, and their settings are explained on the upstream [parameters page](https://ethz-asl.github.io/wavemap/pages/parameters/index.html).
Large pointclouds received over live topics, such as an Ouster's, need suitable DDS middleware settings, or most of them are silently lost. The [DDS page](interfaces/ros2/docs/dds.md) describes them for Fast DDS and Cyclone DDS.

To save the map, run:

```bash
ros2 service call /wavemap/save_map wavemap2_msgs/srv/FilePath "{file_path: /path/to/map.wvmp}"
```

The [usage page](interfaces/ros2/docs/usage.md) covers the launch files' arguments, replaying and batch-processing rosbags, the demos, and the server's other services.

## Visualization

The launch files open Rviz2 with wavemap's map display, which shows the server's map on the `/wavemap/map` topic as it is built.
Pass `show_rviz:=false` on machines without a display.

Saved `.wvmp` maps, including maps saved with ROS1, can be opened without the server: run `rviz2`, add the `wavemap2_rviz_plugin/WavemapMap` display, set its `Source` to `File`, and choose the file under `Loaded map`.
The display's `Render voxels` and `Render slice` options control how the map is drawn.

## Documentation

The ROS2 interface is documented in [`interfaces/ros2/docs`](interfaces/ros2/docs):

* [Installation](interfaces/ros2/docs/installation.md): Docker image and container, VS Code devcontainer, native build, and build options
* [Map message layout](interfaces/ros2/docs/map_message_layout.md): the `flat` and `nested` layouts, choosing between them, and diagnosing mismatches
* [Usage](interfaces/ros2/docs/usage.md): launch files, the server and the rosbag processor, the demos, Rviz2, and services
* [DDS middleware](interfaces/ros2/docs/dds.md): Fast DDS and Cyclone DDS settings for large pointclouds and maps, UDP buffers, and containers
* [Odometry](interfaces/ros2/docs/odometry.md): the TF frames wavemap needs, and iG-LIO
* [Your own code](interfaces/ros2/docs/your_own_code.md): using wavemap's ROS2 packages in your own ROS2 package, with code examples

Everything that is not specific to ROS2 is documented on [wavemap's documentation site](https://ethz-asl.github.io/wavemap/):

* [Parameters](https://ethz-asl.github.io/wavemap/pages/parameters/index.html): all config file settings, which apply unchanged in ROS2
* [Demos](https://ethz-asl.github.io/wavemap/pages/demos.html): the datasets and pre-built maps used by the demos
* [C++ library](https://ethz-asl.github.io/wavemap/pages/tutorials/cpp.html) and [Python package](https://ethz-asl.github.io/wavemap/pages/tutorials/python.html): working with saved maps, with or without ROS
* [ROS1 installation](https://ethz-asl.github.io/wavemap/pages/installation/ros1.html) and [ROS1 interface](https://ethz-asl.github.io/wavemap/pages/tutorials/ros1.html)
* [FAQ](https://ethz-asl.github.io/wavemap/pages/faq.html)

## License and credits

Wavemap is released under the [BSD 3-Clause License](LICENSE), copyright Autonomous Systems Lab, ETH Zurich.
The ROS2 interface was added in this fork by [akhilj95](https://github.com/akhilj95) and is released under the same license.
For wavemap itself, please refer to the [upstream repository](https://github.com/ethz-asl/wavemap).
