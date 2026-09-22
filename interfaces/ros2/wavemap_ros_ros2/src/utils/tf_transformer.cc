#include "wavemap_ros_ros2/utils/tf_transformer.h"

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include <tf2_ros/buffer_interface.hpp>

#include "wavemap_ros_conversions_ros2/geometry_msg_conversions.h"

namespace wavemap {
namespace {
// tf2 measures its cache time as a tf2::Duration, so the config's float
// seconds have to be converted rather than passed through as ROS1 did with
// ros::Duration(float).
tf2::Duration toTf2Duration(FloatingPoint seconds) {
  return std::chrono::duration_cast<tf2::Duration>(
      std::chrono::duration<FloatingPoint>(seconds));
}
}  // namespace

TfTransformer::TfTransformer(rclcpp::Node& node,
                             FloatingPoint tf_buffer_cache_time)
    : tf_buffer_(node.get_clock(), toTf2Duration(tf_buffer_cache_time)),
      tf_listener_(
          std::make_unique<tf2_ros::TransformListener>(tf_buffer_, &node)) {}

TfTransformer::TfTransformer(rclcpp::Clock::SharedPtr clock,
                             FloatingPoint tf_buffer_cache_time)
    : tf_buffer_(std::move(clock), toTf2Duration(tf_buffer_cache_time)),
      tf_listener_(nullptr) {}

bool TfTransformer::isTransformAvailable(
    const std::string& to_frame_id, const std::string& from_frame_id,
    const rclcpp::Time& frame_timestamp) const {
  return tf_buffer_.canTransform(sanitizeFrameId(to_frame_id),
                                 sanitizeFrameId(from_frame_id),
                                 tf2_ros::fromRclcpp(frame_timestamp));
}

bool TfTransformer::waitForTransform(const std::string& to_frame_id,
                                     const std::string& from_frame_id,
                                     const rclcpp::Time& frame_timestamp) {
  return waitForTransformImpl(sanitizeFrameId(to_frame_id),
                              sanitizeFrameId(from_frame_id),
                              tf2_ros::fromRclcpp(frame_timestamp));
}

std::optional<Transformation3D> TfTransformer::lookupLatestTransform(
    const std::string& to_frame_id, const std::string& from_frame_id) {
  // NOTE: tf2 treats the zero time point as "the latest available transform",
  //       exactly as ROS1's default-constructed ros::Time{} did.
  return lookupTransformImpl(sanitizeFrameId(to_frame_id),
                             sanitizeFrameId(from_frame_id),
                             tf2::TimePointZero);
}

std::optional<Transformation3D> TfTransformer::lookupTransform(
    const std::string& to_frame_id, const std::string& from_frame_id,
    const rclcpp::Time& frame_timestamp) {
  return lookupTransformImpl(sanitizeFrameId(to_frame_id),
                             sanitizeFrameId(from_frame_id),
                             tf2_ros::fromRclcpp(frame_timestamp));
}

bool TfTransformer::setTransform(
    const geometry_msgs::msg::TransformStamped& transform_msg,
    const std::string& authority, bool is_static) {
  return tf_buffer_.setTransform(transform_msg, authority, is_static);
}

std::string TfTransformer::sanitizeFrameId(const std::string& string) {
  if (string[0] == '/') {
    return string.substr(1, string.length());
  } else {
    return string;
  }
}

bool TfTransformer::waitForTransformImpl(
    const std::string& to_frame_id, const std::string& from_frame_id,
    const tf2::TimePoint& frame_timestamp) const {
  // Total time spent waiting for the updated pose
  std::chrono::duration<double> t_waited{0.0};
  while (t_waited < transform_lookup_max_time_) {
    if (tf_buffer_.canTransform(to_frame_id, from_frame_id, frame_timestamp)) {
      return true;
    }
    std::this_thread::sleep_for(transform_lookup_retry_period_);
    t_waited += transform_lookup_retry_period_;
  }
  LOG(WARNING) << "Waited " << t_waited.count()
               << "s, but still could not get the TF from " << from_frame_id
               << " to " << to_frame_id << " at timestamp "
               << std::chrono::duration_cast<std::chrono::seconds>(
                      frame_timestamp.time_since_epoch())
                      .count()
               << " seconds";
  return false;
}

std::optional<Transformation3D> TfTransformer::lookupTransformImpl(
    const std::string& to_frame_id, const std::string& from_frame_id,
    const tf2::TimePoint& frame_timestamp) {
  if (!tf_buffer_.canTransform(to_frame_id, from_frame_id, frame_timestamp)) {
    return std::nullopt;
  }
  const geometry_msgs::msg::TransformStamped transform_msg =
      tf_buffer_.lookupTransform(to_frame_id, from_frame_id, frame_timestamp);
  return convert::transformMsgToTransformation3D(transform_msg.transform);
}
}  // namespace wavemap
