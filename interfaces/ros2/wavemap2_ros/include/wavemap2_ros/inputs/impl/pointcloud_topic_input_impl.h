#ifndef WAVEMAP_ROS_ROS2_INPUTS_IMPL_POINTCLOUD_TOPIC_INPUT_IMPL_H_
#define WAVEMAP_ROS_ROS2_INPUTS_IMPL_POINTCLOUD_TOPIC_INPUT_IMPL_H_

#include <glog/logging.h>

namespace wavemap {
namespace detail {
// Recovers the message type from a PointcloudTopicInput::callback overload's
// pointer-to-member type.
//
// ROS1's registrar received a pointer-to-member and handed it straight to
// nh.subscribe(topic, queue, ptr, this), which deduced the message type
// itself. ROS2's create_subscription<MessageT>() takes the message type as an
// explicit template argument instead, so the registrar has to name it. This
// trait lets it, while keeping registerCallback's shape identical to ROS1's.
// Declaration only: it is used solely inside decltype().
template <typename ClassT, typename MessageT>
MessageT pointcloudCallbackMessageType(void (ClassT::*)(const MessageT&));
}  // namespace detail

template <typename RegistrarT>
bool PointcloudTopicInput::registerCallback(PointcloudTopicType type,
                                            RegistrarT registrar) {
  switch (type) {
    case PointcloudTopicType::kPointCloud2:
    case PointcloudTopicType::kOuster:
      // clang-format off
      registrar(static_cast<void(PointcloudTopicInput::*)(
                    const sensor_msgs::msg::PointCloud2&)>(
          &PointcloudTopicInput::callback));
      // clang-format on
      return true;
    case PointcloudTopicType::kLivox:
#ifdef LIVOX_AVAILABLE
      // clang-format off
      registrar(static_cast<void(PointcloudTopicInput::*)(
                    const livox_ros_driver2::msg::CustomMsg&)>(
          &PointcloudTopicInput::callback));
      // clang-format on
      return true;
#else
      LOG(ERROR) << "Livox support is currently not available. Please install "
                    "livox_ros_driver2 and rebuild wavemap.";
      return false;
#endif
    default:
      LOG(ERROR)
          << "Requested callback registration for unknown PointcloudTopicType "
             "\""
          << type.toStr() << "\"";
      return false;
  }
}
}  // namespace wavemap

#endif  // WAVEMAP_ROS_ROS2_INPUTS_IMPL_POINTCLOUD_TOPIC_INPUT_IMPL_H_
