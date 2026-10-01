#ifndef WAVEMAP_ROS_ROS2_UTILS_TF_TRANSFORMER_H_
#define WAVEMAP_ROS_ROS2_UTILS_TF_TRANSFORMER_H_

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <wavemap/core/common.h>

namespace wavemap {
// Port of interfaces/ros1/wavemap_ros/include/wavemap_ros/utils/
// tf_transformer.h.
class TfTransformer {
 public:
  // Subscribes to /tf and /tf_static through a TransformListener, using the
  // node's own clock for the buffer.
  explicit TfTransformer(rclcpp::Node& node,
                         FloatingPoint tf_buffer_cache_time = 10.f);

  // Same TransformListener subscription, but with an explicit buffer clock
  // instead of the node's own. Used by the rosbag processor: it needs
  // use_sim_time, but tf2_ros::Buffer clears every *dynamic* transform (its
  // static cache survives) via Buffer::onTimeJump the moment the node's
  // clock switches from system to ROS time -- i.e. the instant the first
  // /clock message lands. A clock that can never switch type sidesteps that
  // entirely, without giving up the live listener.
  TfTransformer(rclcpp::Node& node, rclcpp::Clock::SharedPtr buffer_clock,
                FloatingPoint tf_buffer_cache_time = 10.f);

  // Check whether a transform is available
  bool isTransformAvailable(const std::string& to_frame_id,
                            const std::string& from_frame_id,
                            const rclcpp::Time& frame_timestamp) const;

  // Waits for a transform to become available, while doing less aggressive
  // polling than ROS's standard tf2_ros::Buffer::canTransform(...)
  bool waitForTransform(const std::string& to_frame_id,
                        const std::string& from_frame_id,
                        const rclcpp::Time& frame_timestamp);

  // Lookup transforms and convert them to Kindr
  std::optional<Transformation3D> lookupTransform(
      const std::string& to_frame_id, const std::string& from_frame_id,
      const rclcpp::Time& frame_timestamp);
  std::optional<Transformation3D> lookupLatestTransform(
      const std::string& to_frame_id, const std::string& from_frame_id);

  // Strip leading slashes if needed to avoid TF errors
  static std::string sanitizeFrameId(const std::string& string);

 private:
  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // Transform lookup timers
  // NOTE: These are deliberately wall-clock, as in ROS1, where they were
  //       ros::WallDuration. They pace a retry loop that waits for another
  //       thread to fill the buffer, so they must keep ticking even when sim
  //       time is paused.
  // Timeout between each update attempt
  static constexpr std::chrono::duration<double> transform_lookup_retry_period_{
      0.02};
  // Maximum time to wait before giving up
  static constexpr std::chrono::duration<double> transform_lookup_max_time_{
      0.25};

  bool waitForTransformImpl(const std::string& to_frame_id,
                            const std::string& from_frame_id,
                            const tf2::TimePoint& frame_timestamp) const;
  std::optional<Transformation3D> lookupTransformImpl(
      const std::string& to_frame_id, const std::string& from_frame_id,
      const tf2::TimePoint& frame_timestamp);
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_ROS2_UTILS_TF_TRANSFORMER_H_
