#include "wavemap_ros_ros2/map_operations/map_ros_operation_factory.h"

#include <memory>
#include <string>
#include <utility>

#include <glog/logging.h>

#include "wavemap_ros_ros2/map_operations/publish_pointcloud_operation.h"

namespace wavemap {
std::unique_ptr<MapOperationBase> MapRosOperationFactory::create(
    const param::Value& params, MapBase::Ptr occupancy_map,
    std::shared_ptr<ThreadPool> thread_pool,
    std::shared_ptr<TfTransformer> transformer, std::string world_frame,
    rclcpp::Node& node) {
  if (const auto type = MapRosOperationType::from(params); type) {
    return create(type.value(), params, std::move(occupancy_map),
                  std::move(thread_pool), std::move(transformer),
                  std::move(world_frame), node);
  }

  LOG(ERROR) << "Could not create map operation. Returning nullptr.";
  return nullptr;
}

std::unique_ptr<MapOperationBase> MapRosOperationFactory::create(
    MapRosOperationType ros_operation_type, const param::Value& params,
    MapBase::Ptr occupancy_map, std::shared_ptr<ThreadPool> thread_pool,
    std::shared_ptr<TfTransformer> transformer, std::string world_frame,
    rclcpp::Node& node) {
  // Unused until phase 4 restores the two operations that need them:
  // publish_map takes the thread pool, crop_map takes the transformer.
  (void)thread_pool;
  (void)transformer;
  if (!ros_operation_type.isValid()) {
    LOG(ERROR) << "Received request to create map operation with invalid type.";
    return nullptr;
  }

  // Create the operation handler
  switch (ros_operation_type) {
    case MapRosOperationType::kPublishPointcloud:
      if (const auto config = PublishPointcloudOperationConfig::from(params);
          config) {
        return std::make_unique<PublishPointcloudOperation>(
            config.value(), std::move(occupancy_map), std::move(world_frame),
            node);
      } else {
        LOG(ERROR) << "Publish pointcloud operation config could not be "
                      "loaded.";
        return nullptr;
      }
    case MapRosOperationType::kPublishMap:
    case MapRosOperationType::kCropMap:
      // Not yet ported. Scheduled for phase 4 (see claude/phases.md).
      //
      // Returning nullptr here is non-fatal by design: RosServer::addOperation
      // simply does not add the stage, and the rest of the pipeline runs. That
      // matters because all eight shipped ROS1 configs list publish_map, so
      // refusing to start on it would mean no stock config could be used
      // unmodified. Map contents are unaffected -- publishing and cropping are
      // both downstream of integration.
      LOG(WARNING) << "Map operation type \"" << ros_operation_type.toStr()
                   << "\" is not yet available in the ROS2 interface. "
                      "Skipping this operation; the rest of the pipeline is "
                      "unaffected.";
      return nullptr;
  }

  LOG(ERROR) << "Factory does not (yet) support creation of map operation type "
             << ros_operation_type.toStr() << ".";
  return nullptr;
}
}  // namespace wavemap
