# Installation

Wavemap's ROS2 packages target [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html) on Ubuntu 24.04 (Noble).
If you are not already using Ubuntu 24.04 and ROS 2 Jazzy, we recommend running wavemap in Docker.

## Docker

If you have not yet installed Docker on your computer, please follow [these instructions](https://docs.docker.com/engine/install/).
We also recommend executing the [post-installation steps for Linux](https://docs.docker.com/engine/install/linux-postinstall/), to make Docker available without `sudo` privileges.

### Building the image

The ROS2 image is built from a local copy of the repository:

```bash
git clone https://github.com/akhilj95/wavemap.git
cd wavemap
docker build --target=workspace-built --tag=wavemap2 -f tooling/docker/ros2/full.Dockerfile .
```

The `--target=workspace-built` argument is required.
Without it, Docker builds the [Dockerfile](../../../tooling/docker/ros2/full.Dockerfile)'s last stage, which only holds the compiler cache.

The image is built with the `flat` [map message layout](map_message_layout.md).
For the `nested` layout, add `--build-arg WAVEMAP2_MSGS_LAYOUT=nested`.

The resulting image contains a built colcon workspace in `/home/ci/ros2_ws`, which the container's shell sources automatically.

### Running a container

To run the container with GUI support, so that Rviz2 can open on your display, use:

```bash
xhost +local:docker
docker run --rm -it --network host \
  -e DISPLAY="${DISPLAY}" -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ~/data:/home/ci/data \
  wavemap2
```

This mounts your `~/data` folder at `/home/ci/data`, where the demos look for their datasets by default.
The `--network host` flag is only needed when ROS2 nodes outside the container, such as sensor drivers or odometry, have to communicate with wavemap.
In that case, both sides also need the same `ROS_DOMAIN_ID` and RMW, see [DDS middleware](dds.md#containers-and-multiple-computers).

**The image uses Cyclone DDS by default, already configured for wavemap's large messages**, instead of ROS 2 Jazzy's default, Fast DDS.
Nodes outside the container that communicate with wavemap must use Cyclone DDS too, and `net.core.rmem_max` must be raised on the host, see [Cyclone DDS](dds.md#cyclone-dds).
To use Fast DDS instead, add `-e RMW_IMPLEMENTATION=rmw_fastrtps_cpp`.

If you have an Nvidia GPU and the [Nvidia container toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html), add `--runtime=nvidia --gpus all -e NVIDIA_DRIVER_CAPABILITIES=graphics` to make Rviz2 much more responsive with large maps.

Since the container is started with `--rm`, it is erased when it exits.
Save maps to a mounted folder, such as `/home/ci/data`, to keep them.
The container runs as root, so files it writes there are owned by root on the host.

### VS Code devcontainer

For development, [`.devcontainer/vscode-ros2`](../../../.devcontainer/vscode-ros2) defines a VS Code devcontainer built on top of the `wavemap2` image, so build the image first.
It mounts your checkout of the repository at `/home/ci/wavemap` and builds the workspace from it, so changes you make to the code are built in the container.
Unlike the image, it runs as a regular user, `ci`, with `sudo` rights.
It uses the image's [Cyclone DDS](dds.md#cyclone-dds) settings.
It does not take a layout option of its own, see [map message layout](map_message_layout.md#devcontainer).

## Native install

Install ROS 2 Jazzy by following the [standard instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
The `ros-jazzy-ros-base` variant is sufficient.

Make sure these required dependencies are installed:

```bash
sudo apt update
sudo apt install git build-essential  # General
sudo apt install python3-rosdep python3-colcon-common-extensions  # ROS
```

Next, clone the code for wavemap:

```bash
cd ~
git clone https://github.com/akhilj95/wavemap.git
```

Unlike for ROS1, the repository must not be cloned into the workspace's `src` folder directly.
Its top-level `CMakeLists.txt` would make colcon treat the whole repository as a single package and skip the ROS2 packages inside it.
Instead, create a colcon workspace and link only the ROS2 packages into it:

```bash
mkdir -p ~/ros2_ws/src
ln -s ~/wavemap/interfaces/ros2 ~/ros2_ws/src/wavemap
```

Make sure rosdep is initialized:

```bash
sudo rosdep init
```

Then install the remaining system dependencies, including Rviz2, using:

```bash
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install -y --from-paths ~/ros2_ws/src --ignore-src
```

Build all of wavemap's ROS2 packages with:

```bash
cd ~/ros2_ws
colcon build --packages-up-to wavemap2_all --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Finally, source the workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

## Build options

* `CMAKE_BUILD_TYPE`: Always build in `Release` mode. Without optimizations, wavemap's code can run up to an order of magnitude slower; we measured 8x for the rosbag processor.
* `WAVEMAP2_MSGS_LAYOUT`: `flat` (default) or `nested`, selects the [map message layout](map_message_layout.md). CMake remembers it between builds of the same workspace. Saved `.wvmp` map files do not depend on the layout, and are the same as in ROS1.
