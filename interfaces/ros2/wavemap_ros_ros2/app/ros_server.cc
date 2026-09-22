#include <memory>

#include <glog/logging.h>
#include <rclcpp/rclcpp.hpp>

#include "wavemap_ros_ros2/ros_server.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  auto node = std::make_shared<rclcpp::Node>("wavemap");
  wavemap::RosServer wavemap_server(*node);

  // NOTE: Single-threaded, and not incidentally. PointcloudTopicInput's
  //       pointcloud_queue_ has no mutex, because ROS1's single-threaded
  //       spinner serialized every callback. A MultiThreadedExecutor here
  //       would race the subscription callback against the retry timer. See
  //       claude/issues.md.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
