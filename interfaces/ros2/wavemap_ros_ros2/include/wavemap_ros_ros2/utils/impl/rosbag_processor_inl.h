#ifndef WAVEMAP_ROS_ROS2_UTILS_IMPL_ROSBAG_PROCESSOR_INL_H_
#define WAVEMAP_ROS_ROS2_UTILS_IMPL_ROSBAG_PROCESSOR_INL_H_

#include <functional>
#include <string>
#include <utility>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

namespace wavemap {
namespace detail {
// Turn a bag's raw bytes back into a typed message.
//
// ROS1's rosbag::MessageInstance::instantiate<T>() did this. rosbag2 hands
// out serialized buffers instead, so the deserialization is explicit.
template <typename MessageT>
MessageT deserializeBagMessage(
    const rosbag2_storage::SerializedBagMessage& bag_message) {
  const rclcpp::SerializedMessage serialized{*bag_message.serialized_data};
  MessageT message;
  rclcpp::Serialization<MessageT>{}.deserialize_message(&serialized, &message);
  return message;
}
}  // namespace detail

template <typename MessageT>
void RosbagProcessor::addCallback(
    const std::string& ros_topic_name,
    std::function<void(const MessageT&)> function_ptr) {
  callbacks_.try_emplace(
      ros_topic_name,
      [function_ptr](const rosbag2_storage::SerializedBagMessage& msg) {
        function_ptr(detail::deserializeBagMessage<MessageT>(msg));
      });
}

template <typename MessageT, typename CallbackObjectT>
void RosbagProcessor::addCallback(
    const std::string& ros_topic_name,
    void (CallbackObjectT::*function_ptr)(const MessageT&),
    CallbackObjectT* object_ptr) {
  callbacks_.try_emplace(
      ros_topic_name,
      [function_ptr,
       object_ptr](const rosbag2_storage::SerializedBagMessage& msg) {
        std::invoke(function_ptr, *object_ptr,
                    detail::deserializeBagMessage<MessageT>(msg));
      });
}
}  // namespace wavemap

#endif  // WAVEMAP_ROS_ROS2_UTILS_IMPL_ROSBAG_PROCESSOR_INL_H_
