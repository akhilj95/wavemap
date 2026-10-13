#include "wavemap2_ros_conversions/config_conversions.h"

#include <limits>
#include <regex>
#include <string>

#include <rclcpp/logging.hpp>

// NOTE: Private header next to this file (see CMakeLists.txt), so it is
//       deliberately included without a directory.
#include "yaml_node_conversions.h"  // NOLINT(build/include_subdir)

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
// Places where the versions disagree, all of which these patterns resolve in
// PyYAML's favour:
//   * `yes`/`no`/`on`/`off` are booleans in 1.1, plain strings in 1.2.
//   * an unsigned exponent such as `1e3` is a *string* in PyYAML, because its
//     float pattern requires a signed exponent. `1.0e+3` is a float in both.
//   * a float without a leading digit takes no sign: `.5` is a float, but
//     `-.5` and `+.5` are strings (`-0.5` is a float).
//
// The patterns were checked against PyYAML 6.0.3 on a table of ~75 scalars.
// Known deviations from PyYAML, none of which appear in wavemap's shipped
// configs:
//   * sexagesimal numbers (`1:30`) and underscore digit separators (`1_000`)
//     are not recognised and fall through to string.
//   * integers outside the range of `int` become floats, since param::Value
//     has no wider integer type.
//   * `null`, `~` and an empty value become an empty string, with a warning,
//     since param::Value has no null.
//   * binary integers (`0b101`) are read by std::stoi, which only understands
//     the `0b` prefix on glibc 2.38 or newer (Ubuntu 24.04, i.e. Jazzy, has
//     it). On older systems they would be read as 0.

// PyYAML omits bare `y`/`n` from its bool resolver, so we do too.
const std::regex kBoolTruePattern{"^(?:yes|Yes|YES|true|True|TRUE|on|On|ON)$"};
const std::regex kBoolFalsePattern{
    "^(?:no|No|NO|false|False|FALSE|off|Off|OFF)$"};
const std::regex kNullPattern{"^(?:~|null|Null|NULL|)$"};
const std::regex kIntPattern{
    "^(?:[-+]?(?:0|[1-9][0-9]*)"  // decimal
    "|[-+]?0b[01]+"               // binary
    "|[-+]?0[0-7]+"               // octal
    "|[-+]?0x[0-9a-fA-F]+)$"};    // hexadecimal
const std::regex kFloatPattern{
    "^(?:[-+]?[0-9]+\\.[0-9]*(?:[eE][-+][0-9]+)?"  // 1.  1.5  1.5e+3
    "|\\.[0-9]+(?:[eE][-+][0-9]+)?)$"};            // .5  .5e-3 (unsigned)
// NOTE: These two cannot go through std::stod, which rejects the leading dot.
const std::regex kInfPattern{"^[-+]?\\.(?:inf|Inf|INF)$"};
const std::regex kNanPattern{"^\\.(?:nan|NaN|NAN)$"};

param::Value scalarToParamValue(const YAML::Node& node,
                                const rclcpp::Logger& logger) {
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
  if (std::regex_match(scalar, kInfPattern)) {
    constexpr double kInf = std::numeric_limits<double>::infinity();
    return param::Value{scalar.front() == '-' ? -kInf : kInf};
  }
  if (std::regex_match(scalar, kNanPattern)) {
    return param::Value{std::numeric_limits<double>::quiet_NaN()};
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
    RCLCPP_WARN(logger,
                "Encountered a null value while parsing YAML config. "
                "wavemap's param type has no null, so it is read as an "
                "empty string.");
    return param::Value{std::string{}};
  }

  return param::Value{scalar};
}
}  // namespace

param::Map toParamMap(const YAML::Node& node,  // NOLINT
                      const rclcpp::Logger& logger) {
  if (!node.IsMap()) {
    RCLCPP_WARN(logger, "Expected param map.");
    return {};
  }

  param::Map param_map;
  for (const auto& kv : node) {
    param_map.emplace(kv.first.Scalar(), toParamValue(kv.second, logger));
  }
  return param_map;
}

param::Array toParamArray(const YAML::Node& node,  // NOLINT
                          const rclcpp::Logger& logger) {
  if (!node.IsSequence()) {
    RCLCPP_WARN(logger, "Expected param array.");
    return {};
  }

  param::Array array;
  array.reserve(node.size());
  for (const auto& element : node) {
    array.emplace_back(toParamValue(element, logger));
  }
  return array;
}

param::Value toParamValue(const YAML::Node& node,  // NOLINT
                          const rclcpp::Logger& logger) {
  switch (node.Type()) {
    case YAML::NodeType::Map:
      return param::Value{toParamMap(node, logger)};
    case YAML::NodeType::Sequence:
      return param::Value{toParamArray(node, logger)};
    case YAML::NodeType::Scalar:
      return scalarToParamValue(node, logger);
    case YAML::NodeType::Null:
      RCLCPP_WARN(logger,
                  "Encountered a null value while parsing YAML config. "
                  "wavemap's param type has no null, so it is read as an "
                  "empty string.");
      return param::Value{std::string{}};
    case YAML::NodeType::Undefined:
    default:
      RCLCPP_ERROR(logger,
                   "Encountered undefined node while parsing YAML config.");
      break;
  }

  // On error, return an empty array, matching the ROS1 converter's behaviour.
  return param::Value{param::Array{}};
}

std::optional<param::Value> yamlStringToParams(const std::string& yaml,
                                               const rclcpp::Logger& logger) {
  try {
    return toParamValue(YAML::Load(yaml), logger);
  } catch (const YAML::Exception& e) {
    RCLCPP_ERROR_STREAM(logger, "Failed to parse YAML config: " << e.what());
    return std::nullopt;
  }
}

std::optional<param::Value> yamlFileToParams(
    const std::filesystem::path& file_path, const rclcpp::Logger& logger) {
  std::error_code error_code;
  if (!std::filesystem::is_regular_file(file_path, error_code)) {
    RCLCPP_ERROR_STREAM(logger, "Config file does not exist: " << file_path);
    return std::nullopt;
  }

  try {
    return toParamValue(YAML::LoadFile(file_path.string()), logger);
  } catch (const YAML::Exception& e) {
    RCLCPP_ERROR_STREAM(logger, "Failed to parse config file "
                                    << file_path << ": " << e.what());
    return std::nullopt;
  }
}
}  // namespace wavemap::param::convert
