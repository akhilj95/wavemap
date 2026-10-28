#ifndef WAVEMAP2_ROS_UTILS_ROS_LOGGING_LEVEL_H_
#define WAVEMAP2_ROS_UTILS_ROS_LOGGING_LEVEL_H_

#include <array>
#include <string>

#include <rcutils/logging.h>
#include <wavemap/core/config/type_selector.h>
#include <wavemap/core/utils/logging_level.h>

namespace wavemap {
// NOTE: The type names are part of the config schema (`general.logging_level`)
//       shared with the ROS1 interface.
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

  // Apply the logger level to a given output
  // NOTE: An empty name addresses rcutils' default logger, which every
  //       node-specific logger falls back to. Callers that have a node pass
  //       its logger name instead.
  bool applyToRosConsole(const std::string& logger_name = "") const;
  void applyToGlog() const;
};
}  // namespace wavemap

#endif  // WAVEMAP2_ROS_UTILS_ROS_LOGGING_LEVEL_H_
