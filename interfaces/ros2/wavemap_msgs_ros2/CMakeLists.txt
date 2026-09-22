cmake_minimum_required(VERSION 3.10)
project(wavemap_msgs_ros2)

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
