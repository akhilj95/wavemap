#ifndef WAVEMAP_ROS_CONVERSIONS_ROS2_SRC_YAML_NODE_CONVERSIONS_H_
#define WAVEMAP_ROS_CONVERSIONS_ROS2_SRC_YAML_NODE_CONVERSIONS_H_

#include <wavemap/core/config/config_base.h>
#include <yaml-cpp/yaml.h>

// Internal header: not installed, so that yaml-cpp remains a private
// dependency of this package. Mirrors the XmlRpcValue overloads in ROS1's
// interfaces/ros1/wavemap_ros_conversions/src/config_conversions.cc.
namespace wavemap::param::convert {
param::Map toParamMap(const YAML::Node& node);
param::Array toParamArray(const YAML::Node& node);
param::Value toParamValue(const YAML::Node& node);
}  // namespace wavemap::param::convert

#endif  // WAVEMAP_ROS_CONVERSIONS_ROS2_SRC_YAML_NODE_CONVERSIONS_H_
