#ifndef WAVEMAP2_ROS_CONVERSIONS_CONFIG_CONVERSIONS_H_
#define WAVEMAP2_ROS_CONVERSIONS_CONFIG_CONVERSIONS_H_

#include <filesystem>
#include <optional>
#include <string>

#include <rclcpp/logger.hpp>
#include <wavemap/core/config/config_base.h>

// Load wavemap's YAML config files as param::Value trees
// NOTE: ROS2's parameter system cannot represent a list of heterogeneous
//       dicts, which wavemap's `map_operations:` and `inputs:` sections both
//       are, so the config files are parsed directly. Their schema is the same
//       as for ROS1, so existing config files work unchanged.
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
    const std::filesystem::path& file_path,
    const rclcpp::Logger& logger =
        rclcpp::get_logger("wavemap2_ros_conversions"));

// As above, for YAML held in memory. Mainly useful for tests.
std::optional<param::Value> yamlStringToParams(
    const std::string& yaml, const rclcpp::Logger& logger = rclcpp::get_logger(
                                 "wavemap2_ros_conversions"));
}  // namespace wavemap::param::convert

#endif  // WAVEMAP2_ROS_CONVERSIONS_CONFIG_CONVERSIONS_H_
