#include "wavemap_ros_ros2/map_operations/publish_map_operation.h"

#include <memory>
#include <string>
#include <utility>

#include <std_srvs/srv/empty.hpp>
#include <wavemap/core/utils/profile/profiler_interface.h>
#include <wavemap_msgs_ros2/msg/map.hpp>

#include "wavemap_ros_conversions_ros2/map_msg_conversions.h"
#include "wavemap_ros_ros2/utils/private_name.h"

namespace wavemap {
DECLARE_CONFIG_MEMBERS(PublishMapOperationConfig,
                      (once_every)
                      (max_num_blocks_per_msg)
                      (topic));

bool PublishMapOperationConfig::isValid(bool verbose) const {
  bool all_valid = true;

  all_valid &= IS_PARAM_GT(once_every, 0.f, verbose);
  all_valid &= IS_PARAM_GT(max_num_blocks_per_msg, 0, verbose);
  all_valid &= IS_PARAM_NE(topic, "", verbose);

  return all_valid;
}

PublishMapOperation::PublishMapOperation(
    const PublishMapOperationConfig& config, MapBase::Ptr occupancy_map,
    std::shared_ptr<ThreadPool> thread_pool, std::string world_frame,
    rclcpp::Node& node)
    : MapOperationBase(std::move(occupancy_map)),
      config_(config.checkValid()),
      thread_pool_(std::move(thread_pool)),
      world_frame_(std::move(world_frame)),
      clock_(node.get_clock()),
      last_run_timestamp_(0, 0, clock_->get_clock_type()) {
  // NOTE: Default QoS, i.e. reliable, matching ROS1's TCP transport. Map
  //       updates are incremental, so a dropped message would leave its blocks
  //       missing downstream until the next full retransmission.
  map_pub_ = node.create_publisher<wavemap_msgs_ros2::msg::Map>(
      privateName(config_.topic), rclcpp::QoS(10));
  republish_whole_map_srv_ = node.create_service<std_srvs::srv::Empty>(
      privateName(config_.topic + "_request_full"),
      [this](const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
             std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/) {
        publishMap(clock_->now(), true);
      });
}

bool PublishMapOperation::shouldRun(const rclcpp::Time& current_time) {
  return config_.once_every < (current_time - last_run_timestamp_).seconds();
}

void PublishMapOperation::run(bool force_run) {
  const rclcpp::Time current_time = clock_->now();
  if (force_run || shouldRun(current_time)) {
    publishMap(current_time, false);
    last_run_timestamp_ = current_time;
  }
}

void PublishMapOperation::publishMap(const rclcpp::Time& current_time,
                                     bool republish_whole_map) {
  ProfilerZoneScoped;
  // If the map is empty, there's no work to do
  if (occupancy_map_->empty()) {
    return;
  }

  if (auto* hashed_wavelet_octree =
          dynamic_cast<HashedWaveletOctree*>(occupancy_map_.get());
      hashed_wavelet_octree) {
    publishHashedMap(current_time, hashed_wavelet_octree, republish_whole_map);
  } else if (auto* hashed_chunked_wavelet_octree =
                 dynamic_cast<HashedChunkedWaveletOctree*>(
                     occupancy_map_.get());
             hashed_chunked_wavelet_octree) {
    publishHashedMap(current_time, hashed_chunked_wavelet_octree,
                     republish_whole_map);
  } else {
    occupancy_map_->threshold();
    wavemap_msgs_ros2::msg::Map map_msg;
    if (convert::mapToRosMsg(*occupancy_map_, world_frame_, current_time,
                             map_msg)) {
      map_pub_->publish(map_msg);
    }
  }
}
}  // namespace wavemap
