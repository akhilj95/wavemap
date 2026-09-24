#ifndef WAVEMAP_ROS_ROS2_MAP_OPERATIONS_CROP_MAP_OPERATION_H_
#define WAVEMAP_ROS_ROS2_MAP_OPERATIONS_CROP_MAP_OPERATION_H_

#include <memory>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <wavemap/core/config/config_base.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap/pipeline/map_operations/map_operation_base.h>

#include "wavemap_ros_ros2/utils/tf_transformer.h"

namespace wavemap {
/**
 * Config struct for map cropping operations.
 */
struct CropMapOperationConfig : public ConfigBase<CropMapOperationConfig, 3> {
  //! Time period controlling how often the map is cropped.
  Seconds<FloatingPoint> once_every = 10.f;

  //! Name of the TF frame to treat as the center point. Usually the robot's
  //! body frame. When the cropper runs, all blocks that are further than
  //! remove_blocks_beyond_distance from this point are deleted.
  std::string body_frame = "body";

  //! Distance beyond which blocks are deleted when the cropper is executed.
  Meters<FloatingPoint> remove_blocks_beyond_distance;

  static MemberMap memberMap;

  bool isValid(bool verbose) const override;
};

class CropMapOperation : public MapOperationBase {
 public:
  // NOTE: Unlike ROS1, this takes the node, for its clock. ROS1 read the
  //       global ros::Time::now(), which ROS2 does not have.
  CropMapOperation(const CropMapOperationConfig& config,
                   MapBase::Ptr occupancy_map,
                   std::shared_ptr<TfTransformer> transformer,
                   std::string world_frame, rclcpp::Node& node);

  bool shouldRun(const rclcpp::Time& current_time);

  void run(bool force_run) override;

 private:
  const CropMapOperationConfig config_;
  const std::shared_ptr<TfTransformer> transformer_;
  const std::string world_frame_;

  // The node's clock and the last run time, typed to match it. See
  // PublishPointcloudOperation for why last_run_timestamp_ must not be
  // default-constructed.
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Time last_run_timestamp_;
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_ROS2_MAP_OPERATIONS_CROP_MAP_OPERATION_H_
