# DDS middleware

ROS 2 Jazzy uses [Fast DDS](https://fast-dds.docs.eprosima.com/) by default, and [Cyclone DDS](https://cyclonedds.io/) is the most common alternative.
We tested wavemap with both.
Wavemap's Docker image uses [Cyclone DDS](#cyclone-dds), already configured.

Two kinds of wavemap's messages are large enough for the middleware's settings to matter:

* **Pointclouds** that wavemap and your odometry receive over live topics, such as an Ouster's, at several MB each. With unsuitable settings, both RMWs can silently lose most of them.
* **Maps** that wavemap publishes, which reach tens of MB. How quickly they are sent depends on the RMW and the [map message layout](map_message_layout.md).

None of this applies to the rosbag processor, which reads its inputs directly from the rosbags.

## Choosing an RMW

| | Fast DDS | Cyclone DDS |
|---|---|---|
| Native install | Installed with ROS 2 Jazzy, and the default | Install `ros-jazzy-rmw-cyclonedds-cpp` and select it |
| Docker image | Installed | Installed, selected and configured by default |
| Large live pointclouds | Need its UDP transport (`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`), which `wavemap_server.launch.xml` sets for wavemap | Work with wavemap's Cyclone DDS config |
| UDP receive buffer setting it follows | `net.core.rmem_default` | `net.core.rmem_max` |
| Map messages, `flat` layout | Fast | Fast |
| Map messages, `nested` layout | Fast | Slow, and mapping pauses while a map is sent |

In our tests on live Ouster data, both delivered every pointcloud once configured as below, at the same CPU usage.
Use the same RMW for all of your nodes, including the sensor driver, the odometry and Rviz2.
We have not tested mixing RMWs.

To select an RMW, set `RMW_IMPLEMENTATION` in every terminal or container that starts ROS2 nodes, before starting them:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  # or rmw_fastrtps_cpp, ROS 2 Jazzy's default
```

After changing it, run `ros2 daemon stop`, so that the `ros2` command line tools do not keep using a daemon started with the previous RMW.

To use Fast DDS in the Docker container instead of Cyclone DDS, add `-e RMW_IMPLEMENTATION=rmw_fastrtps_cpp` to the `docker run` command.

## Fast DDS

By default, Fast DDS delivers messages between processes on the same computer through shared memory.
For large best-effort messages, such as pointclouds, this transport silently drops most of them whenever the receiving node falls behind.

Fast DDS only uses shared memory between two nodes if both of them allow it.
Restricting a node to UDP with `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` therefore makes all of its connections use UDP:

* `wavemap_server.launch.xml` sets it on the `wavemap` node. If you run `ros_server` directly, or write your own launch files around it instead of including `wavemap_server.launch.xml`, set this environment variable yourself.
* Set it for any other node that receives the large pointclouds, such as your odometry, or for the sensor driver that publishes them, which covers all of its subscribers.

Over UDP, Fast DDS does not choose a receive buffer size for its sockets, so they get the system default, `net.core.rmem_default`, which is about 208 KB on Ubuntu.

Raise both `rmem_max` and `rmem_default`, as described [below](#udp-receive-buffers).

## Cyclone DDS

> **The Docker image is already set up for Cyclone DDS.**
> It includes Cyclone DDS's RMW, selects it with `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, and points `CYCLONEDDS_URI` to [wavemap's Cyclone DDS config](../wavemap2_ros/config/cyclonedds.xml), so nothing needs to be installed or set inside the container.
> This also applies to the [VS Code devcontainer](installation.md#vs-code-devcontainer), which is built on the image.
> Two things remain to be done outside the container:
>
> * raise `net.core.rmem_max` on the host, as described [below](#udp-receive-buffers),
> * run nodes outside the container that exchange messages with wavemap, such as your sensor driver, odometry or Rviz2, with Cyclone DDS too, see [containers and multiple computers](#containers-and-multiple-computers).

For a native install, install Cyclone DDS's RMW, and select it and wavemap's config in every terminal that starts ROS2 nodes, after sourcing the workspace:

```bash
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$(ros2 pkg prefix wavemap2_ros)/share/wavemap2_ros/config/cyclonedds.xml
```

Cyclone DDS uses UDP between processes by default, so it does not drop large pointclouds the way Fast DDS's shared-memory transport does, and `FASTDDS_BUILTIN_TRANSPORTS` has no effect on it.

Use the `flat` [map message layout](map_message_layout.md) with Cyclone DDS, which is the default.
With the `nested` layout, Cyclone DDS serializes each map's octree nodes one at a time.
In our tests, publishing a 30 MB map then took about 80 ms instead of about 10 ms with the `flat` layout, and wavemap pauses mapping while it publishes.
Its maps also arrived about 200 ms after they were stamped, against 50 to 65 ms with Fast DDS.
With the `flat` layout, Cyclone DDS delivered maps slightly faster than Fast DDS.

### Wavemap's Cyclone DDS config

Cyclone DDS is configured through an XML file, passed with `CYCLONEDDS_URI`.
[Wavemap's config](../wavemap2_ros/config/cyclonedds.xml), based on the settings [Autoware](https://autowarefoundation.github.io/autoware-documentation/main/installation/additional-settings-for-developers/network-configuration/dds-settings/) uses for large messages, raises the maximum UDP message size to 65500 B and requests a 10 MB receive buffer for each socket:

```xml
<SocketReceiveBufferSize max="10MB"/>
```

The kernel caps the buffer at `net.core.rmem_max`, so raise it to at least 10 MB on the host, as described [below](#udp-receive-buffers).
Since the config sets the buffer size itself, `rmem_default` does not matter to Cyclone DDS.
Without a config, Cyclone DDS only raises buffers smaller than 1 MB to 1 MB, and otherwise keeps `rmem_default`.

Autoware's settings use `min="10MB"` instead of `max="10MB"`.
With `min`, Cyclone DDS refuses to start any node if `rmem_max` is below 10 MB, while with `max` it starts with the largest buffer the kernel allows.
To change the settings, copy the file, edit it, and point `CYCLONEDDS_URI` to your copy, for example with `-e CYCLONEDDS_URI=file:///home/ci/data/cyclonedds.xml` on `docker run`.

## UDP receive buffers

Both RMWs receive large messages over UDP, and the kernel limits how much data each socket can buffer.
To raise the limit (`rmem_max`) and the default (`rmem_default`) to 64 MB, the values we tested with, run:

```bash
sudo sysctl -w net.core.rmem_max=67108864 net.core.rmem_default=67108864
```

These settings are lost when the computer restarts.

`rmem_default` applies to every socket on the computer that does not choose its own size.
With Cyclone DDS and wavemap's config, only `rmem_max` is needed, so `rmem_default` can be left out.
When running wavemap in Docker, set them on the host, since a container cannot change them.

## Containers and multiple computers

Nodes in different Docker containers, or on different computers, can only communicate if:

* they use the same RMW,
* they use the same `ROS_DOMAIN_ID`,
* their network allows it. For Docker containers, use host networking (`--network host`), as in the [container command](installation.md#running-a-container).

Since wavemap's Docker image uses Cyclone DDS, nodes on the host or in other containers that communicate with it must use Cyclone DDS too, as described [above](#cyclone-dds).
Alternatively, switch the container to Fast DDS, as described in [choosing an RMW](#choosing-an-rmw).

## Map message layouts and RMWs

Every node that receives wavemap's maps must be built with the same [map message layout](map_message_layout.md) as the server.
The two RMWs fail differently when the layouts do not match: Fast DDS delivers no maps at all, while Cyclone DDS delivers them but wavemap's conversions reject them.
See [matching layouts](map_message_layout.md#matching-layouts) for how to diagnose this.
