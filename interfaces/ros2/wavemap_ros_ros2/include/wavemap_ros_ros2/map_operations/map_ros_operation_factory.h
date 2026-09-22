#ifndef WAVEMAP_ROS_ROS2_MAP_OPERATIONS_MAP_ROS_OPERATION_FACTORY_H_
#define WAVEMAP_ROS_ROS2_MAP_OPERATIONS_MAP_ROS_OPERATION_FACTORY_H_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <wavemap/core/config/param.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap/core/utils/thread_pool.h>
#include <wavemap/pipeline/map_operations/map_operation_base.h>

#include "wavemap_ros_ros2/map_operations/map_ros_operation_types.h"
#include "wavemap_ros_ros2/utils/tf_transformer.h"

namespace wavemap {
class MapRosOperationFactory {
 public:
  static std::unique_ptr<MapOperationBase> create(
      const param::Value& params, MapBase::Ptr occupancy_map,
      std::shared_ptr<ThreadPool> thread_pool,
      std::shared_ptr<TfTransformer> transformer, std::string world_frame,
      rclcpp::Node& node);

  static std::unique_ptr<MapOperationBase> create(
      MapRosOperationType ros_operation_type, const param::Value& params,
      MapBase::Ptr occupancy_map, std::shared_ptr<ThreadPool> thread_pool,
      std::shared_ptr<TfTransformer> transformer, std::string world_frame,
      rclcpp::Node& node);
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_ROS2_MAP_OPERATIONS_MAP_ROS_OPERATION_FACTORY_H_
