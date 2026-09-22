#ifndef WAVEMAP_ROS_ROS2_UTILS_TF_TRANSFORMER_H_
#define WAVEMAP_ROS_ROS2_UTILS_TF_TRANSFORMER_H_

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <wavemap/core/common.h>

namespace wavemap {
// Port of interfaces/ros1/wavemap_ros/include/wavemap_ros/utils/
// tf_transformer.h.
class TfTransformer {
 public:
  // Online: subscribes to /tf and /tf_static through a TransformListener.
  explicit TfTransformer(rclcpp::Node& node,
                         FloatingPoint tf_buffer_cache_time = 10.f);

  // Offline: a bare buffer with no listener. Transforms are pushed in
  // directly via setTransform(), which is how the rosbag processor feeds TF
  // without a round-trip through the ROS graph. See the note in
  // rosbag_processor.h for why that round-trip is avoided.
  explicit TfTransformer(rclcpp::Clock::SharedPtr clock,
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

  // Insert a transform directly into the buffer, bypassing /tf. Used by the
  // offline rosbag processor; a no-op source of transforms for the live node,
  // which gets them from its TransformListener instead.
  bool setTransform(const geometry_msgs::msg::TransformStamped& transform_msg,
                    const std::string& authority, bool is_static);

  // Strip leading slashes if needed to avoid TF errors
  static std::string sanitizeFrameId(const std::string& string);

 private:
  tf2_ros::Buffer tf_buffer_;
  // Null in offline mode. Held by pointer rather than by value because the
  // listener starts a subscription (and, by default, its own spin thread) on
  // construction, which the offline processor must not do.
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
