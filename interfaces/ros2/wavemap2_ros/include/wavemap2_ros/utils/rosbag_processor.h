#ifndef WAVEMAP_ROS_ROS2_UTILS_ROSBAG_PROCESSOR_H_
#define WAVEMAP_ROS_ROS2_UTILS_ROSBAG_PROCESSOR_H_

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
// Port of interfaces/ros1/wavemap_ros/include/wavemap_ros/utils/
// rosbag_processor.h, on top of rosbag2_cpp::Reader.
//
// TF is handled exactly as in ROS1: /tf and /tf_static are republished from
// the bag onto the live topics of the same name via addRepublisher(), and
// the node's own TransformListener (see TfTransformer) picks them back up,
// same as it would for a live robot. This means the run is not
// bit-reproducible -- whether a given transform has arrived by the time a
// given pointcloud is looked up depends on wall-clock scheduling of the
// round-trip through the ROS graph, exactly as in ROS1 (confirmed: 9 runs of
// this processor on the same bag integrated 4 different cloud counts, see
// progress.md). That was a deliberate, explicit trade against an earlier
// design that injected TF straight into the buffer and was bit-deterministic
// -- reproducing ROS1's exact behavior (including its nondeterminism) was
// judged more valuable than the determinism, since it lets any launch-time
// static transform publisher work unmodified in batch mode too, with no
// generated bag or special-cased static-only path required.
//
// One departure from ROS1 remains: input queues are drained through
// addQueueFlusher() immediately after the messages that filled them, instead
// of waiting for a ros::Timer to fire during spinOnce(). ROS1's timer only
// fired once the simulated /clock this same process published had travelled
// back to it.
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

  // Register work to run after each batch of bag messages, used to drain the
  // inputs' queues. See the class comment.
  void addQueueFlusher(std::function<void()> flusher) {
    queue_flushers_.emplace_back(std::move(flusher));
  }

  // NOTE on transient_local: tf2_ros::TransformListener subscribes to
  // /tf_static requesting TRANSIENT_LOCAL durability. The default VOLATILE
  // publisher this would otherwise create is QoS-incompatible with that
  // subscription -- ros2 bag play hits the exact same mismatch when replaying
  // a bag whose /tf_static was not recorded as transient_local (see
  // issues.md, "/tf_static QoS"). Pass transient_local=true when
  // republishing /tf_static for that reason.
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

#endif  // WAVEMAP_ROS_ROS2_UTILS_ROSBAG_PROCESSOR_H_
