# Wavemap ROS2 interface documentation

These pages document wavemap's ROS2 interface in more detail than the [repository README](../../../README.md), which gives an overview and the basic commands.

* [Installation](installation.md): Docker image and container, VS Code devcontainer, native build, and build options
* [Map message layout](map_message_layout.md): the `flat` and `nested` layouts, choosing between them, and diagnosing mismatches
* [Usage](usage.md): launch files, the server and the rosbag processor, the demos, Rviz2, and services
* [DDS middleware](dds.md): Fast DDS and Cyclone DDS settings for large pointclouds and maps, UDP buffers, and containers
* [Odometry](odometry.md): the TF frames wavemap needs, and iG-LIO
* [Your own code](your_own_code.md): using wavemap's ROS2 packages in your own ROS2 package, with code examples

Everything that is not specific to ROS2, such as the mapping library, the config parameters, the C++ and Python APIs and the ROS1 interface, is documented on [wavemap's documentation site](https://ethz-asl.github.io/wavemap/).
