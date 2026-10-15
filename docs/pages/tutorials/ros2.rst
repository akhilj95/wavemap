ROS2 Interface
##############
.. rstcheck: ignore-roles=gh_file

Wavemap's ROS2 interface targets ROS 2 Jazzy and mirrors the :doc:`ROS1 interface <ros1>`: the same server and rosbag processor, the same topic and service names, and the same YAML config files. In the sections that follow, we start by providing an overview of the packages that are included in wavemap's ROS2 interface. We then discuss how they can be configured to process your own sensor data. Finally, we explain how you can interact with wavemap in your own ROS2 nodes to enable new, custom use cases.

Installation instructions are provided on the :doc:`ROS2 installation page <../installation/ros2>`, which also explains how to run the demos.

Overview of packages
********************
Wavemap's ROS2 interface consists of the following packages:

* ``wavemap2``: An ament wrapper for wavemap's C++ Library
* ``wavemap2_msgs``: Message and service types to transmit wavemap maps over ROS2
* ``wavemap2_ros_conversions``: Conversions between ROS2 and internal wavemap types
* ``wavemap2_ros``: The ROS2 server, rosbag processor, and example config and launch files
* ``wavemap2_rviz_plugin``: Rviz2 plugin to efficiently visualize and inspect wavemap maps
* ``wavemap2_all``: A meta package that builds all of the above packages

The packages carry a ``2`` in their names so that they can be told apart from the ROS1 packages that live in the same repository.

Your own data
*************
The only requirements to run wavemap on your own dataset or robot are:

1. a source of depth measurements,
2. sensor pose (estimates) for each measurement.

We usually use depth measurements from depth cameras or 3D LiDARs, but any source would work as long as a sufficiently good :ref:`projection <configuration_projection_models>` and :ref:`measurement <configuration_measurement_models>` model is available. Wavemap's ROS2 interface natively supports depth image and pointcloud inputs, and automatically queries the sensor poses from the TF tree.

To help you get started quickly, we provide example :gh_file:`config <interfaces/ros2/wavemap2_ros/config>` and ROS2 :gh_file:`launch <interfaces/ros2/wavemap2_ros/launch>` files for various sensor setups and use cases. The config files are identical to the ROS1 ones, so the settings on the :doc:`parameters page <../parameters/index>` apply unchanged. Unlike in ROS1, the config file is always passed explicitly, with the launch files' ``param_file`` argument.

Wavemap subscribes to its depth image and pointcloud inputs with ROS2's sensor data QoS profile (best effort), so it receives data from both best-effort and reliable publishers. When pointclouds are large, for example those of an Ouster LiDAR, also see :ref:`large pointclouds over DDS <installation-ros2-large-pointclouds>`.

Odometry from iG-LIO
====================
Any odometry that publishes its poses to TF can be used. We tested wavemap with the ROS2 port of `iG-LIO <https://github.com/Forestry-Robotics-UC/ig_lio/tree/ros2-jazzy>`__, running live on an Ouster LiDAR and its built-in IMU. Wavemap's maps with it match those built with the ROS1 version of iG-LIO and wavemap. The following points are needed to connect the two:

* **TF chain**: iG-LIO publishes the transform from its ``odom_frame`` (default ``odom``) to its ``base_frame`` (default ``base_link``), where the base frame is the IMU. Set wavemap's ``world_frame`` to iG-LIO's ``odom_frame``. Wavemap also needs the transform from the base frame to the pointcloud's frame, ``os_sensor`` for Ouster LiDARs. Publish it as a static transform, using the same LiDAR-to-IMU extrinsics as iG-LIO's ``t_imu_lidar`` and ``R_imu_lidar``. Replace the zeros below with your translation, and add ``--roll``, ``--pitch`` and ``--yaw`` if the rotation is not identity:

  .. code-block:: bash

      ros2 run tf2_ros static_transform_publisher --x 0.0 --y 0.0 --z 0.0 --frame-id base_link --child-frame-id os_sensor

  Do not set iG-LIO's ``base_frame`` to ``os_imu`` if the Ouster driver already publishes ``os_sensor`` to ``os_imu``, since the ``os_imu`` frame would then have two parents.

* **Motion undistortion**: ``undistort_motion: true`` works with iG-LIO's poses. Until iG-LIO publishes its first pose, wavemap skips the incoming pointclouds and logs a warning for each of them. This is expected.

* **Initialization**: iG-LIO estimates its IMU biases and the gravity direction from the first ``init_count`` IMU measurements (20 by default, i.e. 0.2 s at 100 Hz). If the sensor moves during this period, iG-LIO diverges, and so does wavemap's map. Keep the sensor still when iG-LIO starts, or raise ``init_count`` (we used 300, i.e. 3 s).

* **Transport**: iG-LIO's own pointcloud subscription is affected by the same Fast DDS shared-memory losses as wavemap's. In our tests it silently lost about 20% of an Ouster's pointclouds. Set ``FASTDDS_BUILTIN_TRANSPORTS=UDPv4`` for iG-LIO and the LiDAR driver as well, or use Cyclone DDS. This also applies when they run in a separate Docker container from wavemap, in which case both containers need host networking (``--network host``) and the same ``ROS_DOMAIN_ID``.

Publishing maps
===============
Wavemap's ROS2 server offers the same ways to publish its maps to ROS topics as the ROS1 server, with the same config options. Please refer to the documentation on :ref:`ROS1 map operations <configuration_map_operations_ros1>` for an overview of the available options.

The map is published with reliable QoS. To subscribe to it from your own node, both reliable and best-effort subscriptions work.

Saving maps
===========
The server's map can also be written to disk by calling its ``save_map`` service as follows:

.. code-block:: bash

    ros2 service call /wavemap/save_map wavemap2_msgs/srv/FilePath "{file_path: /path/to/your/map.wvmp}"

Saved maps can subsequently be used :ref:`in C++ <cpp-code-examples-read-map>` (with or without ROS) and :ref:`in Python <python-code-examples-read-map>`. The file format is the same as in ROS1.

Your own code
*************
We now briefly discuss how to set up your own ROS2 package to use wavemap, before proceeding to code examples.

Build configuration
===================
In order to use wavemap's ROS2 packages in your own code, you need to add them as dependencies and configure your CMake targets to use them.

Declare dependencies
--------------------
For illustration purposes, we will assume your package uses ``wavemap2``, ``wavemap2_msgs`` and ``wavemap2_ros_conversions``.

First, declare them as dependencies in your package's `package.xml`:

.. code-block:: XML

    <depend>rclcpp</depend>
    <depend>wavemap2</depend>
    <depend>wavemap2_msgs</depend>
    <depend>wavemap2_ros_conversions</depend>

Next, find them in your package's `CMakeLists.txt`:

.. code-block:: cmake

    # Find dependencies
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(wavemap2 REQUIRED)
    find_package(wavemap2_msgs REQUIRED)
    find_package(wavemap2_ros_conversions REQUIRED)

CMake target setup
------------------
The libraries and executables generated by your code are referred to as CMake targets. They have to be linked to the libraries provided by wavemap's ROS2 packages in order to use them. Linking to these libraries' CMake targets also tells your target where to find their header files:

.. code-block:: cmake

    target_link_libraries(your_target PUBLIC
        rclcpp::rclcpp
        wavemap2::wavemap_core
        ${wavemap2_msgs_TARGETS}
        wavemap2_ros_conversions::wavemap2_ros_conversions)

We **strongly recommend** to also call the ``set_wavemap_target_properties`` function, which ``find_package(wavemap2)`` makes available, to ensure your target's compilation options are compatible with those of wavemap:

.. code-block:: cmake

      set_wavemap_target_properties(your_target)

Unless your package's `CMakeLists.txt` sets a default build type, build it with ``colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release``. Without optimizations, wavemap's code can run up to an order of magnitude slower; we measured 8x for the rosbag processor.

Code examples
=============
Since wavemap's ROS2 interface extends its C++ API, all of the :ref:`C++ API's code examples <cpp-code-examples>` can directly be used in ROS2.

Additionally, the following code can be used to receive maps over a ROS2 topic

.. code-block:: c++

    #include <memory>

    #include <rclcpp/rclcpp.hpp>
    #include <wavemap2_ros_conversions/map_msg_conversions.h>

    // Create a smart pointer that will own the received map
    wavemap::MapBase::Ptr loaded_map;

    // Define the map callback
    void mapCallback(const wavemap2_msgs::msg::Map& msg) {
      // Load the received map
      if (wavemap::convert::rosMsgToMap(msg, loaded_map)) {
        RCLCPP_INFO(rclcpp::get_logger("your_node"),
                    "Received a map that uses %zu bytes",
                    loaded_map->getMemoryUsage());
      }
    }

    // NOTE: We only define loaded_map and mapCallback globally for illustration
    //       purposes. In practice, we recommend defining them as class members.

    int main(int argc, char** argv) {
      // Register your node with ROS
      rclcpp::init(argc, argv);
      auto node = std::make_shared<rclcpp::Node>("your_node");

      // Subscribe to the map published by wavemap's server
      const rclcpp::QoS qos(1);
      auto map_sub = node->create_subscription<wavemap2_msgs::msg::Map>(
          "/wavemap/map", qos, &mapCallback);

      rclcpp::spin(node);
      rclcpp::shutdown();
    }

and maps can be sent over ROS2 with

.. code-block:: c++

    #include <chrono>
    #include <memory>
    #include <string>

    #include <rclcpp/rclcpp.hpp>
    #include <wavemap2_ros_conversions/map_msg_conversions.h>

    int main(int argc, char** argv) {
      // Register your node with ROS
      rclcpp::init(argc, argv);
      auto node = std::make_shared<rclcpp::Node>("your_node");

      // Create an empty map for illustration
      wavemap::HashedWaveletOctreeConfig config;
      wavemap::HashedWaveletOctree map(config);

      // Advertise the ROS topic
      const std::string ros_topic = "~/map";
      const rclcpp::QoS qos(1);
      auto map_pub = node->create_publisher<wavemap2_msgs::msg::Map>(ros_topic, qos);

      // Convert the map to a ROS msg and publish it, once per second
      auto timer = node->create_wall_timer(std::chrono::seconds(1), [&]() {
        wavemap2_msgs::msg::Map map_msg;
        const std::string map_frame = "odom";
        const rclcpp::Time stamp = node->now();
        wavemap::convert::mapToRosMsg(map, map_frame, stamp, map_msg);
        map_pub->publish(map_msg);
      });

      rclcpp::spin(node);
      rclcpp::shutdown();
    }

A message published right after the publisher is created is usually lost, because subscribers take a moment to discover a new publisher. This is why the example keeps publishing on a timer.
