#ifndef WAVEMAP_ROS_ROS2_MAP_OPERATIONS_MAP_ROS_OPERATION_TYPES_H_
#define WAVEMAP_ROS_ROS2_MAP_OPERATIONS_MAP_ROS_OPERATION_TYPES_H_

#include <wavemap/core/config/type_selector.h>

namespace wavemap {
// NOTE: All three names are kept, including the two whose operations are not
//       yet ported, because this list is what a config's `type:` string is
//       matched against. Dropping a name would make RosServer::addOperation
//       fall through to "does not match a known operation type name" and
//       print a misleading error.
struct MapRosOperationType : public TypeSelector<MapRosOperationType> {
  using TypeSelector<MapRosOperationType>::TypeSelector;

  enum Id : TypeId { kPublishMap, kPublishPointcloud, kCropMap };

  static constexpr std::array names = {"publish_map", "publish_pointcloud",
                                       "crop_map"};
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_ROS2_MAP_OPERATIONS_MAP_ROS_OPERATION_TYPES_H_
