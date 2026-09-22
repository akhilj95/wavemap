#ifndef WAVEMAP_ROS_ROS2_UTILS_ROSBAG_PROCESSOR_H_
#define WAVEMAP_ROS_ROS2_UTILS_ROSBAG_PROCESSOR_H_

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <glog/logging.h>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include "wavemap_ros_ros2/utils/tf_transformer.h"

namespace wavemap {
// Port of interfaces/ros1/wavemap_ros/include/wavemap_ros/utils/
// rosbag_processor.h, on top of rosbag2_cpp::Reader.
//
// Two deliberate departures from the ROS1 version, both in service of
// determinism, which is what makes this the parity oracle for test level T2
// (see claude/phases.md):
//
//  1. TF is fed straight into the transformer's buffer via addTfInjector(),
//     rather than being republished onto /tf for the node's own
//     TransformListener to pick back up. The ROS1 round-trip through the ROS
//     graph introduced ordering that depended on subscriber setup timing, and
//     left the run open to contamination by any other /tf publisher on the
//     network.
//  2. Input queues are drained through addQueueFlusher() immediately after the
//     messages that filled them, instead of waiting for a ros::Timer to fire
//     during spinOnce(). ROS1's timer only fired once the simulated /clock
//     this same process published had travelled back to it.
//
// Multiple bags are merged chronologically, which is what ROS1's
// rosbag::View did across its queries. This matters for the Newer College
// setup, where odometry and sensor data live in separate bags and have to
// interleave by timestamp for the TF lookups to resolve.
class RosbagProcessor {
 public:
  explicit RosbagProcessor(rclcpp::Node& node) : node_(node) {}

  void addRosbag(const std::string& rosbag_path);
  bool addRosbags(std::istringstream& rosbag_paths);
  bool bagsContainTopic(const std::string& topic_name);

  template <typename MessageT>
  void addCallback(const std::string& ros_topic_name,
                   std::function<void(const MessageT&)> function_ptr);

  template <typename MessageT, typename CallbackObjectT>
  void addCallback(const std::string& ros_topic_name,
                   void (CallbackObjectT::*function_ptr)(const MessageT&),
                   CallbackObjectT* object_ptr);

  // Route /tf and /tf_static straight into the transformer's buffer.
  void addTfInjector(std::shared_ptr<TfTransformer> transformer);

  // Register work to run after each batch of bag messages, used to drain the
  // inputs' queues. See the class comment.
  void addQueueFlusher(std::function<void()> flusher) {
    queue_flushers_.emplace_back(std::move(flusher));
  }

  void addRepublisher(const std::string& rosbag_topic_name,
                      const std::string& republished_topic_name,
                      const std::string& message_type,
                      unsigned int queue_size);

  void enableSimulatedClock() {
    simulated_clock_pub_ =
        node_.create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);
  }
  void disableSimulatedClock() { simulated_clock_pub_.reset(); }

  bool processAll();

 private:
  rclcpp::Node& node_;

  std::vector<std::unique_ptr<rosbag2_cpp::Reader>> opened_rosbags_;

  using RosbagCallback =
      std::function<void(const rosbag2_storage::SerializedBagMessage&)>;
  std::map<std::string, RosbagCallback, std::less<>> callbacks_;
  std::map<std::string, std::shared_ptr<rclcpp::GenericPublisher>>
      republishers_;
  std::vector<std::function<void()>> queue_flushers_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr
      simulated_clock_pub_;

  // Pop the chronologically next message across all opened bags, or nullptr
  // when they are all exhausted.
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> readNextInOrder();
  // One peeked-at message per bag, so the merge can compare heads without
  // consuming them.
  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> heads_;
};
}  // namespace wavemap

#include "wavemap_ros_ros2/utils/impl/rosbag_processor_inl.h"

#endif  // WAVEMAP_ROS_ROS2_UTILS_ROSBAG_PROCESSOR_H_
