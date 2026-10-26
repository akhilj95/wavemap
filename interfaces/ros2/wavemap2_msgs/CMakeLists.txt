cmake_minimum_required(VERSION 3.10)
project(wavemap2_msgs)

# Default to a Release build.
# NOTE: Each colcon package configures separately, so this needs repeating in
#       every package, not only in wavemap2 (see the note there). Without
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

# Choose how octree nodes are laid out in the map messages
# NOTE: The default 'flat' layout stores each node field in its own primitive
#       array, which DDS serializes in bulk. The 'nested' layout keeps ROS1's
#       WaveletOctreeNode[] arrays. Some RMWs (e.g. Cyclone DDS) serialize
#       these element by element, so publishing a 30 MB map took ~80 ms
#       instead of ~10 ms and blocked the mapping thread meanwhile. Both
#       layouts use the same message names, so publishers and subscribers
#       must be built with the same layout.
set(WAVEMAP2_MSGS_LAYOUT "flat" CACHE STRING
    "Octree node layout of the map messages ('flat' or 'nested')")
set_property(CACHE WAVEMAP2_MSGS_LAYOUT PROPERTY STRINGS flat nested)
if (NOT WAVEMAP2_MSGS_LAYOUT MATCHES "^(flat|nested)$")
  message(FATAL_ERROR "Invalid WAVEMAP2_MSGS_LAYOUT '${WAVEMAP2_MSGS_LAYOUT}'."
      " Valid options are 'flat' and 'nested'.")
endif ()
message(STATUS "Using the '${WAVEMAP2_MSGS_LAYOUT}' map message layout.")

# Gather the messages whose definition depends on the chosen layout
# NOTE: The '<base dir>:<relative path>' form makes rosidl read each file from
#       the layout's directory while installing it as msg/<Name>.msg.
set(LAYOUT_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/msg_layouts/${WAVEMAP2_MSGS_LAYOUT}")
set(LAYOUT_MSGS
    "${LAYOUT_DIR}:msg/HashedWaveletOctreeBlock.msg"
    "${LAYOUT_DIR}:msg/WaveletOctree.msg")
if (WAVEMAP2_MSGS_LAYOUT STREQUAL "nested")
  list(APPEND LAYOUT_MSGS "${LAYOUT_DIR}:msg/WaveletOctreeNode.msg")
endif ()

# Declare the messages and services to be generated
# NOTE: With the 'nested' layout, these mirror interfaces/ros1/wavemap_msgs 1:1.
#       The only difference is that Map.msg refers to std_msgs/Header instead
#       of a bare Header, since ROS2's IDL requires the fully qualified name.
rosidl_generate_interfaces(${PROJECT_NAME}
    msg/HashedBlock.msg
    msg/HashedBlocks.msg
    msg/HashedWaveletOctree.msg
    msg/Index3D.msg
    msg/Map.msg
    msg/OctreeNode.msg
    ${LAYOUT_MSGS}
    srv/FilePath.srv
    DEPENDENCIES std_msgs)

ament_export_dependencies(rosidl_default_runtime std_msgs)

ament_package()
