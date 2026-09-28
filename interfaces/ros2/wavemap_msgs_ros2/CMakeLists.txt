cmake_minimum_required(VERSION 3.10)
project(wavemap_msgs_ros2)

# Default to a Release build.
# NOTE: Each colcon package configures separately, so this needs repeating in
#       every package, not only in wavemap_ros2 (see the note there). Without
#       it a plain `colcon build` compiles this package with no -O3 and no
#       NDEBUG, which made the rosbag processor 8x slower.
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "No build type specified, defaulting to Release.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif ()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Declare the messages and services to be generated
# NOTE: These mirror interfaces/ros1/wavemap_msgs 1:1. The only difference is
#       that Map.msg refers to std_msgs/Header instead of a bare Header, since
#       ROS2's IDL requires the fully qualified name.
rosidl_generate_interfaces(${PROJECT_NAME}
    msg/HashedBlock.msg
    msg/HashedBlocks.msg
    msg/HashedWaveletOctree.msg
    msg/HashedWaveletOctreeBlock.msg
    msg/Index3D.msg
    msg/Map.msg
    msg/OctreeNode.msg
    msg/WaveletOctree.msg
    msg/WaveletOctreeNode.msg
    srv/FilePath.srv
    DEPENDENCIES std_msgs)

ament_export_dependencies(rosidl_default_runtime std_msgs)

ament_package()
