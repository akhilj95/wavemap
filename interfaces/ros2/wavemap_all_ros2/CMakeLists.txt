cmake_minimum_required(VERSION 3.10)
project(wavemap_all_ros2)

# Metapackage that pulls in all of wavemap's ROS2 packages.
# Mirrors interfaces/ros1/wavemap_all, which uses catkin_metapackage().
find_package(ament_cmake REQUIRED)

ament_package()
