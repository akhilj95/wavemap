# Map message layout

Wavemap's ROS2 map messages carry the same maps as its ROS1 messages, but by default lay out their octree nodes differently.
This is the main way in which the ROS2 interface differs from the ROS1 interface for its users.

## The two layouts

Wavemap's map messages can be built in one of two layouts, selected with the `WAVEMAP2_MSGS_LAYOUT` CMake option of the `wavemap2_msgs` package:

* `flat` (default): Each octree's nodes are sent as two arrays of numbers, `node_detail_coefficients` and `node_allocated_children_bitsets`, which DDS serializes in one go.
* `nested` (ROS1 style): Each octree's nodes are sent as an array of `WaveletOctreeNode` messages, exactly as in wavemap's ROS1 messages. Some DDS implementations serialize such arrays one element at a time. In our tests, Cyclone DDS took about 80 ms to publish a 30 MB map in this layout, compared to about 10 ms in the flat layout, and the server's mapping is paused while it publishes.

Both layouts use the same message names and carry the same map.
Choose `nested` if your map messages need to match ROS1's, for example to bridge them to ROS1, to read map topics from rosbags recorded with the nested layout, or because your own code reads the messages' `nodes` fields.

## Selecting a layout

To build with the nested layout, pass the option to colcon:

```bash
colcon build --packages-up-to wavemap2_all --cmake-args -DCMAKE_BUILD_TYPE=Release -DWAVEMAP2_MSGS_LAYOUT=nested
```

For the [Docker image](installation.md#docker), add `--build-arg WAVEMAP2_MSGS_LAYOUT=nested` to the `docker build` command instead.

CMake remembers the option, so later builds of the same workspace keep the last layout that was passed, even without `--cmake-args`.
When the layout of an existing workspace is switched, colcon rebuilds the packages that depend on the messages.
Switching from nested to flat, however, leaves the nested layout's `WaveletOctreeNode` files behind in the `install` folder, so we recommend deleting the `build` and `install` folders before switching.

### Devcontainer

The [VS Code devcontainer](installation.md#vs-code-devcontainer) (`.devcontainer/vscode-ros2`) is built on top of the `wavemap2` image and does not take a layout option of its own.
It inherits the layout the `wavemap2` image was built with: its build step reuses the image's `~/ros2_ws`, and CMake remembers the layout there.
To use the nested layout in the devcontainer, build the `wavemap2` image with the nested build argument first.
If the devcontainer's `build` folder is deleted, its next build falls back to the default, flat layout.

## Matching layouts

Every node that exchanges maps with wavemap, including Rviz2 with wavemap's plugin, must be built with the same layout as the server.
If the layouts differ, the subscriber still connects to the map topic, but cannot read the maps:

* With Fast DDS, it receives no maps at all. Depending on which side uses which layout, Fast DDS prints `sequence size exceeds remaining buffer` or nothing.
* With Cyclone DDS, the maps arrive, but `rosMsgToMap` rejects them and logs a warning that names the subscriber's layout. This includes wavemap's Rviz2 plugin.

In both cases, `ros2 topic info -v /wavemap/map` lists a different `Topic type hash` for the publisher than for the subscriptions.
When it starts, the server logs the layout it publishes in:

```text
Publishing maps on '/wavemap/map' using the 'flat' map message layout (WAVEMAP2_MSGS_LAYOUT).
```

To check which layout a workspace was built with, run:

```bash
ros2 interface show wavemap2_msgs/msg/HashedWaveletOctreeBlock
```

The flat layout lists `node_detail_coefficients` and `node_allocated_children_bitsets`, while the nested layout lists `WaveletOctreeNode[] nodes`.

To check which layout a rosbag's map topic was recorded with, compare its `type_description_hash` in the bag's `metadata.yaml` with the `hash_string` of `wavemap2_msgs/msg/Map` in `$(ros2 pkg prefix wavemap2_msgs)/share/wavemap2_msgs/msg/Map.json`.
If they differ, the bag was recorded with the other layout, or with a version of wavemap whose map messages differ.

Saved `.wvmp` map files do not depend on the layout, and are the same as in ROS1.

## In your own code

Code that converts maps with `mapToRosMsg` and `rosMsgToMap` works with either layout.
Only code that reads or writes the octree nodes in the messages directly depends on it, see [your own code](your_own_code.md#map-message-layouts).
