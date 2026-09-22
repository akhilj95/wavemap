#ifndef WAVEMAP_ROS_ROS2_UTILS_POINTCLOUD_UNDISTORTER_H_
#define WAVEMAP_ROS_ROS2_UTILS_POINTCLOUD_UNDISTORTER_H_

#include <memory>
#include <string>
#include <utility>

#include <wavemap/core/common.h>
#include <wavemap/core/data_structure/pointcloud.h>
#include <wavemap/core/utils/undistortion/stamped_pointcloud.h>

#include "wavemap_ros_ros2/utils/tf_transformer.h"

namespace wavemap {
// Port of interfaces/ros1/wavemap_ros/include/wavemap_ros/utils/
// pointcloud_undistorter.h. Unchanged apart from the TF timestamp type: the
// undistortion maths itself lives in the library
// (wavemap/core/utils/undistortion/), not here.
class PointcloudUndistorter {
 public:
  enum class Result {
    kStartTimeNotInTfBuffer,
    kEndTimeNotInTfBuffer,
    kIntermediateTimeNotInTfBuffer,
    kSuccess
  };

  explicit PointcloudUndistorter(std::shared_ptr<TfTransformer> transformer,
                                 int num_interpolation_intervals_per_cloud)
      : transformer_(std::move(transformer)),
        num_interpolation_intervals_per_cloud_(
            num_interpolation_intervals_per_cloud) {}

  Result undistortPointcloud(
      undistortion::StampedPointcloud& stamped_pointcloud,
      PosedPointcloud<>& undistorted_pointcloud,
      const std::string& fixed_frame);

 private:
  std::shared_ptr<TfTransformer> transformer_;
  const int num_interpolation_intervals_per_cloud_;
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_ROS2_UTILS_POINTCLOUD_UNDISTORTER_H_
