#include "wavemap2_ros/utils/rosbag_processor.h"

#include <memory>
#include <sstream>
#include <string>

#include <glog/logging.h>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <wavemap/core/utils/profile/resource_monitor.h>

#include "wavemap2_ros/inputs/depth_image_topic_input.h"
#include "wavemap2_ros/inputs/pointcloud_topic_input.h"
#include "wavemap2_ros/ros_server.h"

using namespace wavemap;  // NOLINT
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  auto node = std::make_shared<rclcpp::Node>("wavemap");

  // Use a live TransformListener, so that launch-time nodes such as
  // static_transform_publisher also work in batch mode
  // NOTE: The buffer uses its own RCL_SYSTEM_TIME clock rather than the
  //       node's, see TfTransformer's two-arg constructor.
  auto transformer = std::make_shared<TfTransformer>(
      *node, std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));

  // Setup the wavemap server node
  RosServer wavemap_server(*node, transformer);

  // Read the required ROS params
  const std::string rosbag_paths_str =
      node->declare_parameter<std::string>("rosbag_path", "");
  const bool keep_alive = node->declare_parameter<bool>("keep_alive", false);
  // NOTE: Checked here since the launch file cannot make rosbag_path a
  //       required arg (included launch files only count explicitly passed
  //       required args).
  if (rosbag_paths_str.find_first_not_of(" \t\n") == std::string::npos) {
    RCLCPP_ERROR(node->get_logger(),
                 "ROS parameter \"rosbag_path\" is not set. It must list "
                 "the rosbag(s) to process, separated by spaces.");
    return -1;
  }

  // Create the rosbag processor and load the rosbags
  RosbagProcessor rosbag_processor{*node};
  std::istringstream rosbag_paths_ss(rosbag_paths_str);
  if (!rosbag_processor.addRosbags(rosbag_paths_ss)) {
    return -1;
  }

  // Setup input handlers
  size_t input_idx = 0u;
  for (const auto& input : wavemap_server.getInputs()) {
    if (auto pointcloud_input =
            dynamic_cast<PointcloudTopicInput*>(input.get());
        pointcloud_input) {
      PointcloudTopicInput::registerCallback(
          pointcloud_input->getTopicType(),
          [&](auto callback_ptr) {
            rosbag_processor.addCallback(input->getTopicName(), callback_ptr,
                                         pointcloud_input);
          },
          node->get_logger());
    } else if (auto depth_image_input =
                   dynamic_cast<DepthImageTopicInput*>(input.get());
               depth_image_input) {
      rosbag_processor.addCallback<sensor_msgs::msg::Image>(
          input->getTopicName(), &DepthImageTopicInput::callback,
          depth_image_input);
    } else {
      RCLCPP_WARN_STREAM(
          node->get_logger(),
          "Failed to register callback for input number "
              << input_idx << ", with topic \"" << input->getTopicName()
              << "\". Support for inputs of type \"" << input->getType().toStr()
              << "\" is not yet implemented in the rosbag processing "
                 "script.");
    }
    // Drain this input's queue as soon as the messages feeding it have been
    // read, rather than waiting for its retry timer. See rosbag_processor.h.
    rosbag_processor.addQueueFlusher(
        [input_ptr = input.get()]() { input_ptr->processQueueNow(); });
    ++input_idx;
  }

  // Republish the bags' TFs for our own TransformListener to pick up
  // NOTE: /tf_static is republished as transient_local regardless of how the
  //       bags recorded it, see addRepublisher(). Bags whose /tf_static was
  //       recorded as volatile therefore work here, although `ros2 bag play`
  //       silently drops their static transforms.
  rosbag_processor.addRepublisher("/tf", "/tf", "tf2_msgs/msg/TFMessage", 10);
  rosbag_processor.addRepublisher("/tf_static", "/tf_static",
                                  "tf2_msgs/msg/TFMessage", 10,
                                  /*transient_local=*/true);

  if (rosbag_processor.bagsContainTopic("/clock")) {
    rosbag_processor.addRepublisher("/clock", "/clock",
                                    "rosgraph_msgs/msg/Clock", 1);
  } else {
    rosbag_processor.enableSimulatedClock();
  }

  // Start measuring resource usage
  ResourceMonitor resource_monitor;
  resource_monitor.start();

  // Process the rosbag
  if (!rosbag_processor.processAll()) {
    return -1;
  }

  // Finish processing the map
  wavemap_server.getPipeline().runOperations(/*force_run_all*/ true);
  wavemap_server.getMap()->prune();

  // Report the resource usage
  resource_monitor.stop();
  LOG(INFO) << "Processing complete.\nResource usage:\n"
            << resource_monitor.getLastEpisodeResourceUsageStats()
            << "\n* Map size: "
            << wavemap_server.getMap()->getMemoryUsage() / 1024 << " kB\n";

  // NOTE: The map can be saved through the save_map service. Run with
  //       keep_alive:=true and call
  //       `ros2 service call /wavemap/save_map wavemap2_msgs/srv/FilePath
  //        "{file_path: /path/to/map.wvmp}"`.
  if (keep_alive) {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  }

  rclcpp::shutdown();
  return 0;
}
