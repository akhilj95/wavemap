#ifndef WAVEMAP_ROS_ROS2_MAP_OPERATIONS_PUBLISH_POINTCLOUD_OPERATION_H_
#define WAVEMAP_ROS_ROS2_MAP_OPERATIONS_PUBLISH_POINTCLOUD_OPERATION_H_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <wavemap/core/config/config_base.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap/core/utils/time/time.h>
#include <wavemap/pipeline/map_operations/map_operation_base.h>

namespace wavemap {
/**
 * Config struct for obstacle pointcloud publishing operations.
 */
struct PublishPointcloudOperationConfig
    : public ConfigBase<PublishPointcloudOperationConfig, 4> {
  //! Time period controlling how often the pointcloud is published.
  Seconds<FloatingPoint> once_every = 1.f;

  //! Threshold in log odds above which cells are classified as occupied.
  FloatingPoint occupancy_threshold_log_odds = 1e-3f;

  bool only_publish_changed_blocks = true;

  //! Name of the topic the pointcloud will be published on.
  std::string topic = "obstacle_pointcloud";

  static MemberMap memberMap;

  bool isValid(bool verbose) const override;
};

class PublishPointcloudOperation : public MapOperationBase {
 public:
  PublishPointcloudOperation(const PublishPointcloudOperationConfig& config,
                             MapBase::Ptr occupancy_map,
                             std::string world_frame, rclcpp::Node& node);

  bool shouldRun(const rclcpp::Time& current_time);

  void run(bool force_run) override;

 private:
  const PublishPointcloudOperationConfig config_;
  const std::string world_frame_;

  // The node's clock, held so that run() can read the time under whichever
  // clock the node uses (sim time during bag replay, system time otherwise).
  rclcpp::Clock::SharedPtr clock_;
  // NOTE: Explicitly constructed with the node clock's type, not
  //       default-constructed. ROS1 used a default ros::Time{} here, but a
  //       default rclcpp::Time is RCL_SYSTEM_TIME, and ROS2 throws on
  //       subtracting times of differing clock types -- so `current_time -
  //       last_run_timestamp_` in shouldRun() would throw on the very first
  //       call. See claude/issues.md.
  rclcpp::Time last_run_timestamp_;

  // Pointcloud publishing
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  Timestamp last_run_timestamp_internal_;
  void publishPointcloud(const rclcpp::Time& current_time);
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_ROS2_MAP_OPERATIONS_PUBLISH_POINTCLOUD_OPERATION_H_
