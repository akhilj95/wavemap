#ifndef WAVEMAP2_ROS_UTILS_POINTCLOUD_MSG_BUILDER_H_
#define WAVEMAP2_ROS_UTILS_POINTCLOUD_MSG_BUILDER_H_

#include <vector>

#include <geometry_msgs/msg/point32.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

namespace wavemap {
// Pack a list of points into a PointCloud2 message, with FLOAT32 x, y and z
// fields at offsets 0, 4 and 8, and a point_step of 12
// NOTE: The points are written directly rather than through a
//       sensor_msgs/PointCloud and convertPointCloudToPointCloud2(), since
//       PointCloud is deprecated.
sensor_msgs::msg::PointCloud2 toPointCloud2Msg(
    const std::vector<geometry_msgs::msg::Point32>& points,
    const std_msgs::msg::Header& header);
}  // namespace wavemap

#endif  // WAVEMAP2_ROS_UTILS_POINTCLOUD_MSG_BUILDER_H_
