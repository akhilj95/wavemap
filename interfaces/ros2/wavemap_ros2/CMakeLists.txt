cmake_minimum_required(VERSION 3.10)
project(wavemap_ros2 VERSION 2.2.1 LANGUAGES CXX)

# Default to a Release build.
# NOTE: Unlike catkin, colcon has no persistent workspace-level config, so a
#       plain `colcon build` would otherwise configure with an empty
#       CMAKE_BUILD_TYPE: no -O3, and no NDEBUG, which leaves the library's
#       DCHECKs live in its hottest loops. The ROS1 side avoids this through
#       `catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release`, set once per
#       workspace. See claude/phases.md.
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "No build type specified, defaulting to Release.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif ()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Build the wavemap library from library/cpp, suppressing its own install
# rules so that this package installs and exports the targets ament-style.
# Mirrors interfaces/ros1/wavemap/CMakeLists.txt.
# Install headers under include/${PROJECT_NAME}/, per ament convention, so
# they cannot collide with another package's in a merged-install or system
# prefix. add_wavemap_include_directories() in the library's wavemap-extras
# bakes ${CMAKE_INSTALL_INCLUDEDIR} into each target's INSTALL_INTERFACE, and
# GNUInstallDirs only defaults it when undefined, so presetting it here
# redirects the exported include path without modifying library/cpp.
# Consumers' #include <wavemap/...> lines are unaffected either way.
set(CMAKE_INSTALL_INCLUDEDIR include/${PROJECT_NAME})

set(GENERATE_WAVEMAP_INSTALL_RULES OFF)
# Map ament's BUILD_TESTING onto the library's own test switch
# (library/cpp/CMakeLists.txt gates tests on ENABLE_TESTING).
if (BUILD_TESTING)
  set(ENABLE_TESTING ON)
endif ()

# Resolve our own location through any symlinks before walking up to the
# library. The documented ROS2 workspace layout symlinks interfaces/ros2 into
# the colcon workspace (decisions.md 7), so CMAKE_CURRENT_SOURCE_DIR is
# typically <ws>/src/wavemap/wavemap_ros2 and walking up from it unresolved
# would land outside the repo entirely.
get_filename_component(WAVEMAP_PACKAGE_DIR "${CMAKE_CURRENT_SOURCE_DIR}"
    REALPATH)
get_filename_component(WAVEMAP_LIBRARY_DIR
    "${WAVEMAP_PACKAGE_DIR}/../../../library/cpp" REALPATH)
if (NOT EXISTS "${WAVEMAP_LIBRARY_DIR}/CMakeLists.txt")
  message(FATAL_ERROR
      "Could not locate the wavemap library at '${WAVEMAP_LIBRARY_DIR}'. "
      "This package must stay inside the wavemap repository, at "
      "interfaces/ros2/wavemap_ros2.")
endif ()
add_subdirectory(${WAVEMAP_LIBRARY_DIR} ${CMAKE_CURRENT_BINARY_DIR}/wavemap)

# Support installs
install(TARGETS wavemap_core wavemap_io wavemap_pipeline
    EXPORT ${PROJECT_NAME}-targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

# Install the library's headers, preserving the wavemap/... include prefix so
# that downstream code is source-identical to ROS1 and pure-CMake users.
install(DIRECTORY ${WAVEMAP_LIBRARY_DIR}/include/wavemap/
    DESTINATION include/${PROJECT_NAME}/wavemap
    FILES_MATCHING PATTERN "*.h")

# Install the library's CMake helpers next to our config extras, so that
# set_wavemap_target_properties() is available to downstream packages.
install(FILES ${WAVEMAP_LIBRARY_DIR}/cmake/wavemap-extras.cmake
    DESTINATION share/${PROJECT_NAME}/cmake)

# Exported as wavemap_ros2::wavemap_core etc, using ament's default
# namespace. Deliberately NOT wavemap::, which would clash with the imported
# targets of the plain CMake wavemap package if a downstream project happened
# to load both (CMake errors on a duplicate imported target).
ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include/${PROJECT_NAME})

ament_package(CONFIG_EXTRAS cmake/wavemap_ros2-extras.cmake)
