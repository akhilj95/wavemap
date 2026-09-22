#ifndef WAVEMAP_ROS_CONVERSIONS_ROS2_TIME_CONVERSIONS_H_
#define WAVEMAP_ROS_CONVERSIONS_ROS2_TIME_CONVERSIONS_H_

#include <cstdint>

#include <rclcpp/time.hpp>

namespace wavemap::convert {
// NOTE: The clock type is explicit, and defaults to RCL_ROS_TIME rather than
//       rclcpp::Time's own RCL_SYSTEM_TIME default. These timestamps are fed
//       straight into tf2 lookups (see the pointcloud undistorter and
//       pointcloud topic input), and rclcpp throws when times of differing
//       clock types are compared or subtracted. Since rclcpp::Time built from
//       a message header is RCL_ROS_TIME, defaulting to anything else here
//       would make timestamps that came in over a topic incomparable with
//       timestamps we reconstruct from nanoseconds.
inline rclcpp::Time nanoSecondsToRosTime(
    uint64_t nsec, rcl_clock_type_t clock_type = RCL_ROS_TIME) {
  return rclcpp::Time{static_cast<int64_t>(nsec), clock_type};
}

inline double nanoSecondsToSeconds(uint64_t nsec) {
  constexpr double kNsecToSec = 1e-9;
  return static_cast<double>(nsec) * kNsecToSec;
}

inline uint64_t rosTimeToNanoSeconds(const rclcpp::Time& time) {
  return static_cast<uint64_t>(time.nanoseconds());
}
}  // namespace wavemap::convert

#endif  // WAVEMAP_ROS_CONVERSIONS_ROS2_TIME_CONVERSIONS_H_
