#ifndef WAVEMAP_ROS_ROS2_UTILS_POINTCLOUD_MSG_BUILDER_H_
#define WAVEMAP_ROS_ROS2_UTILS_POINTCLOUD_MSG_BUILDER_H_

#include <vector>

#include <geometry_msgs/msg/point32.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

namespace wavemap {
// Pack a list of points into a PointCloud2 message.
//
// ROS1 built a sensor_msgs::PointCloud and called
// sensor_msgs::convertPointCloudToPointCloud2() on it. That function still
// exists in Jazzy, but sensor_msgs/PointCloud has been deprecated since Foxy,
// so using it warns on every build and will eventually stop compiling. It
// also copies every point twice.
//
// The message this produces is byte-identical to what the deprecated
// conversion produced for a channel-less PointCloud, which is what both call
// sites had: fields x@0, y@4, z@8, all FLOAT32 with count 1, point_step 12,
// height 1, width N, row_step 12*N, is_bigendian false, is_dense false.
sensor_msgs::msg::PointCloud2 toPointCloud2Msg(
    const std::vector<geometry_msgs::msg::Point32>& points,
    const std_msgs::msg::Header& header);
}  // namespace wavemap

#endif  // WAVEMAP_ROS_ROS2_UTILS_POINTCLOUD_MSG_BUILDER_H_
