#ifndef WAVEMAP_ROS_ROS2_UTILS_ROS_LOGGING_LEVEL_H_
#define WAVEMAP_ROS_ROS2_UTILS_ROS_LOGGING_LEVEL_H_

#include <array>
#include <string>

#include <rcutils/logging.h>
#include <wavemap/core/config/type_selector.h>
#include <wavemap/core/utils/logging_level.h>

namespace wavemap {
// Port of interfaces/ros1/wavemap_ros/include/wavemap_ros/utils/
// ros_logging_level.h. The type names and their YAML spellings are part of the
// config schema (`general.logging_level`), so they are unchanged; only the
// backend differs: rosconsole is replaced by rcutils' logging severities.
struct RosLoggingLevel : public TypeSelector<RosLoggingLevel> {
  using TypeSelector<RosLoggingLevel>::TypeSelector;

  enum Id : TypeId { kDebug, kInfo, kWarning, kError, kFatal };

  static constexpr std::array names = {"debug", "info", "warning", "error",
                                       "fatal"};
  static constexpr std::array ros_levels = {
      RCUTILS_LOG_SEVERITY_DEBUG, RCUTILS_LOG_SEVERITY_INFO,
      RCUTILS_LOG_SEVERITY_WARN, RCUTILS_LOG_SEVERITY_ERROR,
      RCUTILS_LOG_SEVERITY_FATAL};

  // Conversion to general LoggingLevel (from the C++ Library)
  operator LoggingLevel() const;  // NOLINT

  // Apply the logger level to a given output.
  // NOTE: ROS1 defaulted to ROSCONSOLE_DEFAULT_NAME, which is package-scoped.
  //       ROS2 has no equivalent: loggers are named after the node. An empty
  //       name addresses rcutils' default logger, which is the setting every
  //       node-specific logger falls back to, so it is the closest analogue to
  //       "wavemap's own log output". Callers that have a node pass its
  //       logger name instead.
  bool applyToRosConsole(const std::string& logger_name = "") const;
  void applyToGlog() const;
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_ROS2_UTILS_ROS_LOGGING_LEVEL_H_
