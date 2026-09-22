#include "wavemap_ros_ros2/utils/ros_logging_level.h"

#include <string>

#include <glog/logging.h>

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
  // NOTE: ROS1's ros::console::set_logger_level(...) had to be followed by
  //       notifyLoggerLevelsChanged() to take effect. rcutils applies the
  //       severity immediately, so there is no second call here.
  const rcutils_ret_t result = rcutils_logging_set_logger_level(
      logger_name.c_str(), ros_levels[toTypeId()]);
  if (result != RCUTILS_RET_OK) {
    LOG(WARNING) << "Could not set ROS logging level for logger \""
                 << logger_name << "\": " << rcutils_get_error_string().str;
    rcutils_reset_error();
    return false;
  }
  return true;
}
}  // namespace wavemap
