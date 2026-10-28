#ifndef INTERFACES_ROS2_WAVEMAP2_ROS_CONVERSIONS_SRC_YAML_NODE_CONVERSIONS_H_
#define INTERFACES_ROS2_WAVEMAP2_ROS_CONVERSIONS_SRC_YAML_NODE_CONVERSIONS_H_

#include <rclcpp/logger.hpp>
#include <wavemap/core/config/config_base.h>
#include <yaml-cpp/yaml.h>

// Internal header: not installed, so that yaml-cpp remains a private
// dependency of this package.
namespace wavemap::param::convert {
param::Map toParamMap(const YAML::Node& node, const rclcpp::Logger& logger);
param::Array toParamArray(const YAML::Node& node, const rclcpp::Logger& logger);
param::Value toParamValue(const YAML::Node& node, const rclcpp::Logger& logger);
}  // namespace wavemap::param::convert

#endif  // INTERFACES_ROS2_WAVEMAP2_ROS_CONVERSIONS_SRC_YAML_NODE_CONVERSIONS_H_
