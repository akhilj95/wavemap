# Pulled in by ament_package(CONFIG_EXTRAS ...), so this runs in the scope of
# every downstream package that does find_package(wavemap_ros2).
#
# It mirrors library/cpp/cmake/wavemap-config.cmake.in, which serves the same
# purpose for pure-CMake consumers, and the manual dependency shims in
# interfaces/ros1/wavemap/CMakeLists.txt for catkin consumers.

include(CMakeFindDependencyMacro)

find_dependency(Eigen3 REQUIRED NO_MODULE)
find_dependency(Boost 1.71 QUIET COMPONENTS headers)

# wavemap only needs Boost's Preprocessor component, which it links as
# Boost::preprocessor. Boost's system package exposes all headers through a
# single Boost::headers target instead, so alias one to the other.
if (TARGET Boost::headers AND NOT TARGET Boost::preprocessor)
  set_target_properties(Boost::headers PROPERTIES IMPORTED_GLOBAL TRUE)
  add_library(Boost::preprocessor ALIAS Boost::headers)
endif ()

find_dependency(glog QUIET)

# Provides set_wavemap_target_properties(), which every target linking against
# wavemap must call. It applies -march=native PUBLIC, so a target that skips it
# mixes SIMD widths with the library and misaligns Eigen types at runtime.
include("${CMAKE_CURRENT_LIST_DIR}/wavemap-extras.cmake")
