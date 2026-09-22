#include "wavemap_ros_ros2/utils/pointcloud_msg_builder.h"

#include <vector>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace wavemap {
sensor_msgs::msg::PointCloud2 toPointCloud2Msg(
    const std::vector<geometry_msgs::msg::Point32>& points,
    const std_msgs::msg::Header& header) {
  sensor_msgs::msg::PointCloud2 msg;
  msg.header = header;

  sensor_msgs::PointCloud2Modifier modifier(msg);
  // NOTE: Fields are listed out rather than using
  //       setPointCloud2FieldsByString(1, "xyz"), which looks equivalent but
  //       is not: it pads point_step out to 16 bytes, whereas ROS1's
  //       conversion produced a tightly packed 12. Verified by diffing the
  //       two messages, not assumed.
  modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                "z", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(points.size());
  // NOTE: resize() sets height to 1, width to the point count and row_step
  //       accordingly. is_bigendian and is_dense keep their default-
  //       constructed value of false, matching what the deprecated
  //       PointCloud conversion wrote.

  sensor_msgs::PointCloud2Iterator<float> it(msg, "x");
  for (const auto& point : points) {
    it[0] = point.x;
    it[1] = point.y;
    it[2] = point.z;
    ++it;
  }

  return msg;
}
}  // namespace wavemap
