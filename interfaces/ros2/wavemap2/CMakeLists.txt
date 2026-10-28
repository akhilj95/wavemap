cmake_minimum_required(VERSION 3.10)
project(wavemap2 VERSION 2.2.1 LANGUAGES CXX)

# Default to a Release build
# NOTE: colcon has no persistent workspace-level config, so a plain
#       `colcon build` would otherwise configure with an empty
#       CMAKE_BUILD_TYPE: no -O3, and no NDEBUG, which leaves the library's
#       DCHECKs live in its hottest loops.
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "No build type specified, defaulting to Release.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif ()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install headers under include/${PROJECT_NAME}/, per ament convention
# NOTE: The library bakes ${CMAKE_INSTALL_INCLUDEDIR} into each target's
#       INSTALL_INTERFACE, and GNUInstallDirs only defaults it when undefined,
#       so presetting it here redirects the exported include path without
#       modifying library/cpp. Consumers' #include <wavemap/...> lines are
#       unaffected.
set(CMAKE_INSTALL_INCLUDEDIR include/${PROJECT_NAME})

# Load the library, with this package handling its installation
set(GENERATE_WAVEMAP_INSTALL_RULES OFF)
# Map ament's BUILD_TESTING onto the library's own test switch
if (BUILD_TESTING)
  set(ENABLE_TESTING ON)
endif ()
# NOTE: The workspace usually reaches this package through a symlink to
#       interfaces/ros2, so its path is resolved before walking up to the
#       library. Otherwise ../../.. would land outside the repository.
get_filename_component(WAVEMAP_PACKAGE_DIR "${CMAKE_CURRENT_SOURCE_DIR}"
    REALPATH)
get_filename_component(WAVEMAP_LIBRARY_DIR
    "${WAVEMAP_PACKAGE_DIR}/../../../library/cpp" REALPATH)
if (NOT EXISTS "${WAVEMAP_LIBRARY_DIR}/CMakeLists.txt")
  message(FATAL_ERROR
      "Could not locate the wavemap library at '${WAVEMAP_LIBRARY_DIR}'. "
      "This package must stay inside the wavemap repository, at "
      "interfaces/ros2/wavemap2.")
endif ()
add_subdirectory(${WAVEMAP_LIBRARY_DIR} ${CMAKE_CURRENT_BINARY_DIR}/wavemap)

# Support installs
install(TARGETS wavemap_core wavemap_io wavemap_pipeline
    EXPORT ${PROJECT_NAME}-targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

# Install the library's headers, keeping their wavemap/... include prefix
install(DIRECTORY ${WAVEMAP_LIBRARY_DIR}/include/wavemap/
    DESTINATION include/${PROJECT_NAME}/wavemap
    FILES_MATCHING PATTERN "*.h")

# Install the library's CMake helpers, for set_wavemap_target_properties()
install(FILES ${WAVEMAP_LIBRARY_DIR}/cmake/wavemap-extras.cmake
    DESTINATION share/${PROJECT_NAME}/cmake)

# Export the targets as wavemap2::wavemap_core etc
# NOTE: Not as wavemap::, which would clash with the imported targets of the
#       plain CMake wavemap package if a downstream project loaded both.
ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include/${PROJECT_NAME})

ament_package(CONFIG_EXTRAS cmake/wavemap2-extras.cmake)
