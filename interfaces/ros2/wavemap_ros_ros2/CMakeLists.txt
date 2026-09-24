cmake_minimum_required(VERSION 3.10)
project(wavemap_ros_ros2)

# Find dependencies
# NOTE: OpenCV is normally included through cv_bridge, but we still explicitly
#       include it to avoid issues on Jetson. Mirrors the ROS1 package.
find_package(OpenCV REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(wavemap_ros2 REQUIRED)
find_package(wavemap_msgs_ros2 REQUIRED)
find_package(wavemap_ros_conversions_ros2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Optional dependencies
# Mirrors interfaces/ros1/wavemap_ros/CMakeLists.txt:24-28. A missing Livox
# driver must degrade to a skipped feature, never to a build break.
find_package(livox_ros_driver2 QUIET)
if (livox_ros_driver2_FOUND)
  message(STATUS "Found livox_ros_driver2: enabling Livox pointcloud input.")
endif ()

# Libraries
# SHARED for the same reason as wavemap_ros_conversions_ros2: add_library()
# defaults to STATIC, and a static library propagates even its PRIVATE
# dependencies into consumers' link lines.
add_library(${PROJECT_NAME} SHARED
    src/inputs/depth_image_topic_input.cc
    src/inputs/pointcloud_topic_input.cc
    src/inputs/ros_input_base.cc
    src/inputs/ros_input_factory.cc
    src/map_operations/crop_map_operation.cc
    src/map_operations/map_ros_operation_factory.cc
    src/map_operations/publish_map_operation.cc
    src/map_operations/publish_pointcloud_operation.cc
    src/utils/pointcloud_msg_builder.cc
    src/utils/pointcloud_undistorter.cc
    src/utils/ros_logging_level.cc
    src/utils/rosbag_processor.cc
    src/utils/tf_transformer.cc
    src/ros_server.cc)
# Applies -march=native and C++17, PUBLIC. Required on every target that
# links wavemap: mixing SIMD widths misaligns Eigen types at runtime, not at
# compile time.
set_wavemap_target_properties(${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME} PUBLIC
    wavemap_ros2::wavemap_core
    wavemap_ros2::wavemap_io
    wavemap_ros2::wavemap_pipeline
    wavemap_ros_conversions_ros2::wavemap_ros_conversions_ros2
    ${wavemap_msgs_ros2_TARGETS}
    rclcpp::rclcpp
    tf2_ros::tf2_ros
    ${tf2_msgs_TARGETS}
    image_transport::image_transport
    cv_bridge::cv_bridge
    ${OpenCV_LIBS}
    rosbag2_cpp::rosbag2_cpp
    rosbag2_storage::rosbag2_storage
    ${rosgraph_msgs_TARGETS}
    ${std_srvs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${visualization_msgs_TARGETS})
if (livox_ros_driver2_FOUND)
  target_compile_definitions(${PROJECT_NAME} PUBLIC LIVOX_AVAILABLE)
  target_link_libraries(${PROJECT_NAME} PUBLIC ${livox_ros_driver2_TARGETS})
endif ()

# Binaries
add_executable(ros_server app/ros_server.cc)
set_wavemap_target_properties(ros_server)
target_link_libraries(ros_server ${PROJECT_NAME})

add_executable(rosbag_processor app/rosbag_processor.cc)
set_wavemap_target_properties(rosbag_processor)
target_link_libraries(rosbag_processor ${PROJECT_NAME})

# Support installs
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}-targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
install(TARGETS ros_server rosbag_processor
    DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/${PROJECT_NAME}
    DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(
    wavemap_ros2 wavemap_msgs_ros2 wavemap_ros_conversions_ros2
    rclcpp tf2_ros tf2_msgs image_transport cv_bridge OpenCV
    rosbag2_cpp rosbag2_storage rosgraph_msgs
    std_srvs sensor_msgs geometry_msgs visualization_msgs)

ament_package()
