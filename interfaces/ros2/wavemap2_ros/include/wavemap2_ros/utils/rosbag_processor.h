#ifndef WAVEMAP2_ROS_UTILS_ROSBAG_PROCESSOR_H_
#define WAVEMAP2_ROS_UTILS_ROSBAG_PROCESSOR_H_

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <glog/logging.h>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

namespace wavemap {
// Reads rosbags with rosbag2_cpp::Reader and feeds their messages to wavemap
//
// Multiple bags are merged chronologically, so that e.g. odometry and sensor
// data recorded in separate bags interleave for the TF lookups to resolve.
//
// /tf and /tf_static are republished from the bags via addRepublisher(), and
// picked back up by the node's own TransformListener (see TfTransformer), as
// for a live robot. This lets launch-time static transform publishers work in
// batch mode too, but makes runs non-reproducible: whether a transform has
// arrived by the time a pointcloud is looked up depends on how the round-trip
// through the ROS graph is scheduled.
//
// Input queues are drained through addQueueFlusher() right after the messages
// that filled them, rather than by the inputs' retry timers, which would only
// fire once the simulated /clock published by this process reached its own
// node.
class RosbagProcessor {
 public:
  explicit RosbagProcessor(rclcpp::Node& node) : node_(node) {}
  ~RosbagProcessor();

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

  // Register work to run after each batch of bag messages, used to drain the
  // inputs' queues. See the class comment.
  void addQueueFlusher(std::function<void()> flusher) {
    queue_flushers_.emplace_back(std::move(flusher));
  }

  // NOTE: tf2_ros::TransformListener subscribes to /tf_static with
  //       TRANSIENT_LOCAL durability, which a VOLATILE publisher cannot serve.
  //       Pass transient_local=true when republishing /tf_static.
  void addRepublisher(const std::string& rosbag_topic_name,
                      const std::string& republished_topic_name,
                      const std::string& message_type, unsigned int queue_size,
                      bool transient_local = false);

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
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr simulated_clock_pub_;

  // Pop the chronologically next message across all opened bags, or nullptr
  // when they are all exhausted.
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> readNextInOrder();
  // One peeked-at message per bag, so the merge can compare heads without
  // consuming them.
  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> heads_;
};
}  // namespace wavemap

#include "wavemap2_ros/utils/impl/rosbag_processor_inl.h"

#endif  // WAVEMAP2_ROS_UTILS_ROSBAG_PROCESSOR_H_
