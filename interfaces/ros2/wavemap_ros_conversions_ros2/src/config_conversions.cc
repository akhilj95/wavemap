#include "wavemap_ros_conversions_ros2/config_conversions.h"

#include "yaml_node_conversions.h"

#include <regex>
#include <string>

#include <glog/logging.h>

namespace wavemap::param::convert {
namespace {
// Scalar type resolution deliberately follows PyYAML's YAML 1.1 implicit
// resolver rather than yaml-cpp's own YAML 1.2 behaviour.
//
// This is not an arbitrary preference. ROS1 loaded these same files through
// roslaunch's <rosparam> tag, which parses with PyYAML and pushes the result
// onto the parameter server. So "what PyYAML would have produced" is exactly
// "what the ROS1 converter received", and matching it is what makes a ROS1
// user's config behave identically here.
//
// The two places the versions disagree, both of which these patterns resolve
// in PyYAML's favour:
//   * `yes`/`no`/`on`/`off` are booleans in 1.1, plain strings in 1.2.
//   * an unsigned exponent such as `1e3` is a *string* in PyYAML, because its
//     float pattern requires a signed exponent. `1.0e+3` is a float in both.
//
// Known deviations from PyYAML, none of which appear in wavemap's shipped
// configs: sexagesimal numbers (`1:30`) and underscore digit separators
// (`1_000`) are not recognised and fall through to string.

// PyYAML omits bare `y`/`n` from its bool resolver, so we do too.
const std::regex kBoolTruePattern{"^(?:yes|Yes|YES|true|True|TRUE|on|On|ON)$"};
const std::regex kBoolFalsePattern{
    "^(?:no|No|NO|false|False|FALSE|off|Off|OFF)$"};
const std::regex kNullPattern{"^(?:~|null|Null|NULL|)$"};
const std::regex kIntPattern{
    "^(?:[-+]?(?:0|[1-9][0-9]*)"     // decimal
    "|[-+]?0b[01]+"                  // binary
    "|[-+]?0[0-7]+"                  // octal
    "|[-+]?0x[0-9a-fA-F]+)$"};       // hexadecimal
const std::regex kFloatPattern{
    "^(?:[-+]?[0-9]+\\.[0-9]*(?:[eE][-+][0-9]+)?"  // 1.  1.5  1.5e+3
    "|[-+]?\\.[0-9]+(?:[eE][-+][0-9]+)?"           // .5  .5e-3
    "|[-+]?\\.(?:inf|Inf|INF)"                     // .inf
    "|\\.(?:nan|NaN|NAN))$"};                      // .nan

param::Value scalarToParamValue(const YAML::Node& node) {
  const std::string& scalar = node.Scalar();

  // A quoted scalar is always a string, whatever it looks like. yaml-cpp
  // reports the non-specific tag "!" for quoted scalars and "?" for plain
  // ones, which is how we tell `off` from `"off"`.
  if (node.Tag() != "?") {
    return param::Value{scalar};
  }

  if (std::regex_match(scalar, kBoolTruePattern)) {
    return param::Value{true};
  }
  if (std::regex_match(scalar, kBoolFalsePattern)) {
    return param::Value{false};
  }
  if (std::regex_match(scalar, kIntPattern)) {
    try {
      // Bases other than 10 carry their own prefix, which std::stoi reads
      // when given base 0.
      return param::Value{std::stoi(scalar, nullptr, 0)};
    } catch (const std::exception&) {
      // Out of int range. Fall through and let it be read as a float, which
      // is what PyYAML's arbitrary-precision ints degrade to here anyway.
    }
  }
  if (std::regex_match(scalar, kFloatPattern) ||
      std::regex_match(scalar, kIntPattern)) {
    try {
      return param::Value{std::stod(scalar)};
    } catch (const std::exception&) {
      // Fall through to string.
    }
  }
  if (std::regex_match(scalar, kNullPattern)) {
    LOG(WARNING) << "Encountered a null value while parsing YAML config. "
                    "wavemap's param type has no null, so it is read as an "
                    "empty string.";
    return param::Value{std::string{}};
  }

  return param::Value{scalar};
}
}  // namespace

param::Map toParamMap(const YAML::Node& node) {  // NOLINT
  if (!node.IsMap()) {
    LOG(WARNING) << "Expected param map.";
    return {};
  }

  param::Map param_map;
  for (const auto& kv : node) {
    param_map.emplace(kv.first.Scalar(), toParamValue(kv.second));
  }
  return param_map;
}

param::Array toParamArray(const YAML::Node& node) {  // NOLINT
  if (!node.IsSequence()) {
    LOG(WARNING) << "Expected param array.";
    return {};
  }

  param::Array array;
  array.reserve(node.size());
  for (const auto& element : node) {
    array.emplace_back(toParamValue(element));
  }
  return array;
}

param::Value toParamValue(const YAML::Node& node) {  // NOLINT
  switch (node.Type()) {
    case YAML::NodeType::Map:
      return param::Value{toParamMap(node)};
    case YAML::NodeType::Sequence:
      return param::Value{toParamArray(node)};
    case YAML::NodeType::Scalar:
      return scalarToParamValue(node);
    case YAML::NodeType::Null:
      LOG(WARNING) << "Encountered a null value while parsing YAML config. "
                      "wavemap's param type has no null, so it is read as an "
                      "empty string.";
      return param::Value{std::string{}};
    case YAML::NodeType::Undefined:
    default:
      LOG(ERROR) << "Encountered undefined node while parsing YAML config.";
      break;
  }

  // On error, return an empty array, matching the ROS1 converter's behaviour.
  return param::Value{param::Array{}};
}

std::optional<param::Value> yamlStringToParams(const std::string& yaml) {
  try {
    return toParamValue(YAML::Load(yaml));
  } catch (const YAML::Exception& e) {
    LOG(ERROR) << "Failed to parse YAML config: " << e.what();
    return std::nullopt;
  }
}

std::optional<param::Value> yamlFileToParams(
    const std::filesystem::path& file_path) {
  std::error_code error_code;
  if (!std::filesystem::is_regular_file(file_path, error_code)) {
    LOG(ERROR) << "Config file does not exist: " << file_path;
    return std::nullopt;
  }

  try {
    return toParamValue(YAML::LoadFile(file_path.string()));
  } catch (const YAML::Exception& e) {
    LOG(ERROR) << "Failed to parse config file " << file_path << ": "
               << e.what();
    return std::nullopt;
  }
}
}  // namespace wavemap::param::convert
