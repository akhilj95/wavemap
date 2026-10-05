cmake_minimum_required(VERSION 3.10)
project(wavemap2_rviz_plugin)

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
find_package(wavemap2 REQUIRED)
find_package(wavemap2_msgs REQUIRED)
find_package(wavemap2_ros_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_srvs REQUIRED)
# Use the same Qt major version as rviz_common, so they are compatible
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# This setting causes Qt's "MOC" generation to happen automatically. The
# headers containing Q_OBJECT are listed as sources below so AUTOMOC sees them.
set(CMAKE_AUTOMOC ON)

# Libraries
# SHARED explicitly: pluginlib loads it at runtime with dlopen.
add_library(${PROJECT_NAME} SHARED
    include/${PROJECT_NAME}/wavemap_map_display.h
    include/${PROJECT_NAME}/visuals/voxel_visual.h
    include/${PROJECT_NAME}/visuals/slice_visual.h
    include/${PROJECT_NAME}/visuals/cell_selector.h
    include/${PROJECT_NAME}/utils/button_property.h
    src/utils/color_conversions.cc
    src/utils/listeners.cc
    src/visuals/slice_visual.cc
    src/visuals/voxel_visual.cc
    src/visuals/cell_layer.cc
    src/visuals/cell_selector.cc
    src/wavemap_map_display.cc)
# Applies -march=native and C++17, PUBLIC, exactly as every wavemap target
# must. Skipping it mixes SIMD widths with the library and misaligns Eigen
# types at runtime rather than at compile time.
set_wavemap_target_properties(${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME} PUBLIC
    wavemap2::wavemap_core
    wavemap2::wavemap_io
    wavemap2_ros_conversions::wavemap2_ros_conversions
    ${wavemap2_msgs_TARGETS}
    rclcpp::rclcpp
    rviz_common::rviz_common
    rviz_rendering::rviz_rendering
    rviz_ogre_vendor::OgreMain
    pluginlib::pluginlib
    ${std_srvs_TARGETS}
    Qt5::Widgets)

# Avoid Qt signals and slots defining "emit", "slots", etc.
# because they can conflict with boost signals.
target_compile_definitions(${PROJECT_NAME} PUBLIC -DQT_NO_KEYWORDS)

# Disable some warnings coming from Qt code in combination with C++17
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-pedantic -Wno-register)

# Register the display with RViz2
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

# Support installs
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}-targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME})
# RViz2 looks up a display's icon at share/<package>/icons/classes/<Class>.png
install(DIRECTORY icons/
    DESTINATION share/${PROJECT_NAME}/icons)

ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(
    wavemap2 wavemap2_msgs wavemap2_ros_conversions rclcpp
    rviz_common rviz_rendering rviz_ogre_vendor pluginlib std_srvs Qt5)

ament_package()
