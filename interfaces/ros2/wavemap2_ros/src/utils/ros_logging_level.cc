#include "wavemap2_ros/utils/ros_logging_level.h"

#include <string>

#include <rclcpp/logging.hpp>

namespace wavemap {
RosLoggingLevel::operator LoggingLevel() const {
  if (id_ == Id::kDebug) {
    return LoggingLevel::kInfo;
  } else if (Id::kInfo < id_ && id_ <= Id::kFatal) {
    return id_ - 1;
  } else {
    return LoggingLevel::kInvalidTypeId;
  }
}

void RosLoggingLevel::applyToGlog() const {
  operator LoggingLevel().applyToGlog();
}

bool RosLoggingLevel::applyToRosConsole(const std::string& logger_name) const {
  const rcutils_ret_t result = rcutils_logging_set_logger_level(
      logger_name.c_str(), ros_levels[toTypeId()]);
  if (result != RCUTILS_RET_OK) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger(logger_name),
                       "Could not set ROS logging level for logger \""
                           << logger_name
                           << "\": " << rcutils_get_error_string().str);
    rcutils_reset_error();
    return false;
  }
  return true;
}
}  // namespace wavemap
