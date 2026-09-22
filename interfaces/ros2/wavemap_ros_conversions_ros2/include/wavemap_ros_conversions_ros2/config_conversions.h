#ifndef WAVEMAP_ROS_CONVERSIONS_ROS2_CONFIG_CONVERSIONS_H_
#define WAVEMAP_ROS_CONVERSIONS_ROS2_CONFIG_CONVERSIONS_H_

#include <filesystem>
#include <optional>
#include <string>

#include <wavemap/core/config/config_base.h>

// Replaces ROS1's XmlRpc-based converter
// (interfaces/ros1/wavemap_ros_conversions/src/config_conversions.cc).
//
// ROS1 got arbitrarily nested config for free: roslaunch's <rosparam file>
// tag loads a whole YAML tree onto the parameter server as an
// XmlRpc::XmlRpcValue. ROS2's parameter system cannot represent a list of
// heterogeneous dicts, which wavemap's `map_operations:` and `inputs:`
// sections both are, so we parse the YAML file ourselves and hand the
// resulting tree to the same param::Value API the ROS1 path used.
//
// The YAML schema is unchanged: a ROS1 user's existing config file works
// as-is.
//
// NOTE: yaml-cpp is deliberately absent from this header. The node-level
//       overloads live in src/yaml_node_conversions.h, which is not
//       installed, so yaml-cpp stays a private dependency and does not leak
//       into the link interface of everything that consumes this package.
namespace wavemap::param::convert {
// Parse a wavemap YAML config file into a param tree.
// Returns std::nullopt when the file cannot be read or does not parse, so the
// caller can report the failure rather than silently proceeding with an empty
// config.
std::optional<param::Value> yamlFileToParams(
    const std::filesystem::path& file_path);

// As above, for YAML held in memory. Mainly useful for tests.
std::optional<param::Value> yamlStringToParams(const std::string& yaml);
}  // namespace wavemap::param::convert

#endif  // WAVEMAP_ROS_CONVERSIONS_ROS2_CONFIG_CONVERSIONS_H_
