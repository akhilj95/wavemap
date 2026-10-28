#include "wavemap2_ros/ros_server.h"

#include <memory>

#include <glog/logging.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  auto node = std::make_shared<rclcpp::Node>("wavemap");
  wavemap::RosServer wavemap_server(*node);

  // NOTE: Single-threaded on purpose. The inputs' queues have no mutex, so a
  //       MultiThreadedExecutor would race their subscription callbacks
  //       against their retry timers.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
