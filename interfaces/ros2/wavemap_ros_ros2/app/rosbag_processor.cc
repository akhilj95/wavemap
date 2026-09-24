#include <memory>
#include <sstream>
#include <string>

#include <glog/logging.h>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <wavemap/core/utils/profile/resource_monitor.h>

#include "wavemap_ros_ros2/inputs/depth_image_topic_input.h"
#include "wavemap_ros_ros2/inputs/pointcloud_topic_input.h"
#include "wavemap_ros_ros2/ros_server.h"
#include "wavemap_ros_ros2/utils/rosbag_processor.h"

using namespace wavemap;  // NOLINT
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  auto node = std::make_shared<rclcpp::Node>("wavemap");

  // Build a transformer with no TransformListener, fed directly from the bag.
  //
  // NOTE: The clock is a standalone RCL_SYSTEM_TIME clock, deliberately not
  //       node->get_clock(). tf2_ros::Buffer registers a jump callback with
  //       on_clock_change set, and Buffer::onTimeJump clears the buffer when
  //       ROS time is activated -- which, under use_sim_time, happens the
  //       moment the first /clock message lands. Measured behaviour of that
  //       clear: transforms inserted as static survive it (tf2's static cache
  //       ignores clearList()), but every dynamic transform is dropped. That
  //       is the damaging half here, because a dataset's odometry is dynamic
  //       TF -- for Newer College Cloister, a 138Hz camera_init->
  //       imu_forward_prop stream that is the only thing tying the sensor to
  //       the world frame.
  //
  //       Offline lookups always pass an explicit timestamp and never block,
  //       so the buffer's own clock is never actually consulted. A clock that
  //       cannot change type sidesteps the whole problem.
  auto transformer = std::make_shared<TfTransformer>(
      std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));

  // Setup the wavemap server node
  RosServer wavemap_server(*node, transformer);

  // Read the required ROS params
  const std::string rosbag_paths_str =
      node->declare_parameter<std::string>("rosbag_path", "");
  const bool keep_alive = node->declare_parameter<bool>("keep_alive", false);

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
          pointcloud_input->getTopicType(), [&](auto callback_ptr) {
            rosbag_processor.addCallback(input->getTopicName(), callback_ptr,
                                         pointcloud_input);
          });
    } else if (auto depth_image_input =
                   dynamic_cast<DepthImageTopicInput*>(input.get());
               depth_image_input) {
      rosbag_processor.addCallback<sensor_msgs::msg::Image>(
          input->getTopicName(), &DepthImageTopicInput::callback,
          depth_image_input);
    } else {
      LOG(WARNING) << "Failed to register callback for input number "
                   << input_idx << ", with topic \"" << input->getTopicName()
                   << "\". Support for inputs of type \""
                   << input->getType().toStr()
                   << "\" is not yet implemented in the rosbag processing "
                      "script.";
    }
    // Drain this input's queue as soon as the messages feeding it have been
    // read, rather than waiting for its retry timer. See rosbag_processor.h.
    rosbag_processor.addQueueFlusher(
        [input_ptr = input.get()]() { input_ptr->processQueueNow(); });
    ++input_idx;
  }

  // Feed TFs straight into the transformer's buffer, rather than republishing
  // them onto /tf for our own listener to read back. ROS1 had to take the
  // round-trip; we do not, and skipping it removes both a source of ordering
  // nondeterminism and the chance of a foreign /tf publisher joining in.
  rosbag_processor.addTfInjector(transformer);

  if (!rosbag_processor.bagsContainTopic("/clock")) {
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

  // NOTE: The map is written out through the save_map service, exactly as in
  //       ROS1. Run with keep_alive:=true and call
  //       `ros2 service call /wavemap/save_map wavemap_msgs_ros2/srv/FilePath
  //        "{file_path: /path/to/map.wvmp}"`.
  if (keep_alive) {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  }

  rclcpp::shutdown();
  return 0;
}
