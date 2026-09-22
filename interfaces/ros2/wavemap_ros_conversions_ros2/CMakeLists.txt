cmake_minimum_required(VERSION 3.10)
project(wavemap_ros_conversions_ros2)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(wavemap_ros2 REQUIRED)
find_package(wavemap_msgs_ros2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

# Libraries
# SHARED explicitly: add_library() would otherwise default to STATIC, and a
# static library has to propagate even its PRIVATE dependencies to consumers
# for linking, which CMake records as $<LINK_ONLY:...> in the exported
# interface. That would put yaml-cpp back into every consumer's link line.
add_library(${PROJECT_NAME} SHARED
    src/config_conversions.cc
    src/map_msg_conversions.cc
    src/time_conversions.cc)
# Applies -march=native and C++17, PUBLIC, exactly as every wavemap target
# must. Skipping it mixes SIMD widths with the library and misaligns Eigen
# types at runtime rather than at compile time.
set_wavemap_target_properties(${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME} PUBLIC
    wavemap_ros2::wavemap_core
    wavemap_ros2::wavemap_io
    wavemap_ros2::wavemap_pipeline
    ${wavemap_msgs_ros2_TARGETS}
    rclcpp::rclcpp
    ${geometry_msgs_TARGETS}
    ${std_msgs_TARGETS})
# PRIVATE: yaml-cpp is an implementation detail. It appears only in
# src/yaml_node_conversions.h, which is not installed, so it must not end up
# in this target's exported link interface.
target_link_libraries(${PROJECT_NAME} PRIVATE yaml-cpp::yaml-cpp)
target_include_directories(${PROJECT_NAME} PRIVATE src)

# Tests
if (BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # The ROS1 configs double as test data: parity means a ROS1 user's existing
  # file must parse unchanged. Resolved through symlinks, since the
  # documented workspace layout symlinks interfaces/ros2 into src/.
  # Resolve our own path through the symlink BEFORE walking up, otherwise
  # ../.. lands in the workspace's src/ rather than in the repo.
  get_filename_component(THIS_PACKAGE_DIR "${CMAKE_CURRENT_SOURCE_DIR}"
      REALPATH)
  get_filename_component(ROS1_CONFIG_DIR
      "${THIS_PACKAGE_DIR}/../../ros1/wavemap_ros/config" REALPATH)
  get_filename_component(WAVEMAP_LIBRARY_TEST_INCLUDE_DIR
      "${THIS_PACKAGE_DIR}/../../../library/cpp/test/include" REALPATH)

  ament_add_gtest(test_${PROJECT_NAME}
      test/src/test_config_conversions.cc)
  set_wavemap_target_properties(test_${PROJECT_NAME})
  target_compile_definitions(test_${PROJECT_NAME} PRIVATE
      kRos1ConfigDir="${ROS1_CONFIG_DIR}")
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})

  # Typed round-trip test (map -> msg -> map) over all 4 map types, ported
  # from interfaces/ros1/wavemap_ros_conversions/test. Reuses the library's
  # own test fixtures (FixtureBase, GeometryGenerator, ConfigGenerator)
  # rather than duplicating them, same as the ROS1 CMakeLists does.
  # TIMEOUT: the default is 60s, and this binary sits right on it -- the
  # HashedBlocks instantiation alone accounts for ~50s of it, because it is
  # the one dense, non-hierarchical map type and the round-trip has to walk
  # every cell. Measured, not guessed. Without this the test intermittently
  # reports "did not generate a result file" on slower machines while every
  # assertion in it actually passes.
  ament_add_gtest(test_map_msg_${PROJECT_NAME}
      test/src/test_map_msg_conversions.cc
      TIMEOUT 300)
  set_wavemap_target_properties(test_map_msg_${PROJECT_NAME})
  target_include_directories(test_map_msg_${PROJECT_NAME} PRIVATE
      "${WAVEMAP_LIBRARY_TEST_INCLUDE_DIR}")
  target_link_libraries(test_map_msg_${PROJECT_NAME} ${PROJECT_NAME})
endif ()

# Support installs
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}-targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
install(DIRECTORY include/${PROJECT_NAME}
    DESTINATION include/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(
    wavemap_ros2 wavemap_msgs_ros2 rclcpp geometry_msgs std_msgs)

ament_package()
