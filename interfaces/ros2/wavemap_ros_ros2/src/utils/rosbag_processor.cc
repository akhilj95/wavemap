#include "wavemap_ros_ros2/utils/rosbag_processor.h"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>

#include <tf2_msgs/msg/tf_message.hpp>
#include <wavemap/core/utils/profile/profiler_interface.h>

namespace wavemap {
namespace {
constexpr const char* kTfTopic = "/tf";
constexpr const char* kTfStaticTopic = "/tf_static";
}  // namespace

void RosbagProcessor::addRosbag(const std::string& rosbag_path) {
  auto reader = std::make_unique<rosbag2_cpp::Reader>();
  try {
    reader->open(rosbag_path);
  } catch (const std::exception& e) {
    LOG(ERROR) << "Could not open rosbag " << rosbag_path << ": " << e.what();
    return;
  }
  opened_rosbags_.emplace_back(std::move(reader));
  heads_.emplace_back(nullptr);
  LOG(INFO) << "Loaded rosbag " << rosbag_path;
}

bool RosbagProcessor::addRosbags(std::istringstream& rosbag_paths) {
  std::string rosbag_path;
  const size_t num_bags_before = opened_rosbags_.size();
  size_t num_requested = 0u;
  while (rosbag_paths >> rosbag_path) {
    ++num_requested;
    addRosbag(rosbag_path);
    if (!rclcpp::ok()) {
      return false;
    }
  }
  // NOTE: ROS1's rosbag::Bag constructor threw on a missing file, ending the
  //       program. rosbag2's exception is caught in addRosbag so that all bad
  //       paths get reported at once, but a failure must still be a failure.
  return opened_rosbags_.size() - num_bags_before == num_requested;
}

bool RosbagProcessor::bagsContainTopic(const std::string& topic_name) {
  return std::any_of(
      opened_rosbags_.cbegin(), opened_rosbags_.cend(),
      [&topic_name](const std::unique_ptr<rosbag2_cpp::Reader>& reader) {
        const auto& topics = reader->get_metadata().topics_with_message_count;
        return std::any_of(topics.cbegin(), topics.cend(),
                           [&topic_name](const auto& topic) {
                             return topic.topic_metadata.name == topic_name;
                           });
      });
}

void RosbagProcessor::addTfInjector(
    std::shared_ptr<TfTransformer> transformer) {
  // The authority string is what tf2 reports when it complains about
  // conflicting or extrapolated transforms, so it is worth making it say
  // where the transforms actually came from.
  auto inject = [transformer](const tf2_msgs::msg::TFMessage& tf_msg,
                              bool is_static) {
    for (const auto& transform : tf_msg.transforms) {
      transformer->setTransform(transform, "rosbag", is_static);
    }
  };
  addCallback<tf2_msgs::msg::TFMessage>(
      kTfTopic, [inject](const tf2_msgs::msg::TFMessage& tf_msg) {
        inject(tf_msg, /*is_static*/ false);
      });
  addCallback<tf2_msgs::msg::TFMessage>(
      kTfStaticTopic, [inject](const tf2_msgs::msg::TFMessage& tf_msg) {
        inject(tf_msg, /*is_static*/ true);
      });
}

void RosbagProcessor::addRepublisher(const std::string& rosbag_topic_name,
                                     const std::string& republished_topic_name,
                                     const std::string& message_type,
                                     unsigned int queue_size) {
  // NOTE: A generic publisher, so the message type does not have to be known
  //       at compile time. ROS1 templated this on the message type; rosbag2
  //       already carries the type name in its metadata, so it need not be.
  republishers_.try_emplace(
      rosbag_topic_name,
      node_.create_generic_publisher(republished_topic_name, message_type,
                                     rclcpp::QoS(queue_size)));
}

std::shared_ptr<rosbag2_storage::SerializedBagMessage>
RosbagProcessor::readNextInOrder() {
  // Refill any empty heads
  for (size_t bag_idx = 0u; bag_idx < opened_rosbags_.size(); ++bag_idx) {
    if (!heads_[bag_idx] && opened_rosbags_[bag_idx]->has_next()) {
      heads_[bag_idx] = opened_rosbags_[bag_idx]->read_next();
    }
  }

  // Pick the chronologically oldest head
  size_t oldest_idx = opened_rosbags_.size();
  for (size_t bag_idx = 0u; bag_idx < heads_.size(); ++bag_idx) {
    if (!heads_[bag_idx]) {
      continue;
    }
    if (oldest_idx == opened_rosbags_.size() ||
        heads_[bag_idx]->recv_timestamp < heads_[oldest_idx]->recv_timestamp) {
      oldest_idx = bag_idx;
    }
  }
  if (oldest_idx == opened_rosbags_.size()) {
    return nullptr;
  }

  return std::exchange(heads_[oldest_idx], nullptr);
}

bool RosbagProcessor::processAll() {
  ProfilerZoneScoped;
  rcutils_time_point_value_t side_tasks_last_timestamp = 0;
  constexpr rcutils_time_point_value_t kSideTasksDtNsec = 10000000;  // 0.01s

  // Give the subscribers and rest of the system some time to set up
  for (int idx = 0; idx < 5; ++idx) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    rclcpp::spin_some(node_.get_node_base_interface());
  }

  while (const auto msg = readNextInOrder()) {
    // Exit if CTRL+C was pressed
    if (!rclcpp::ok()) {
      return false;
    }

    // Handle callbacks
    if (auto it = callbacks_.find(msg->topic_name); it != callbacks_.end()) {
      ProfilerZoneScopedN("rosbagMsgHandleCallbacks");
      it->second(*msg);
    }

    // Handle republishing
    if (auto it = republishers_.find(msg->topic_name);
        it != republishers_.end()) {
      ProfilerZoneScopedN("rosbagMsgPublishToTopic");
      rclcpp::SerializedMessage serialized{*msg->serialized_data};
      it->second->publish(serialized);
    }

    // Catch up on some periodic tasks
    if (msg->recv_timestamp < side_tasks_last_timestamp ||
        side_tasks_last_timestamp + kSideTasksDtNsec < msg->recv_timestamp) {
      side_tasks_last_timestamp = msg->recv_timestamp;

      // Publish clock substitute if needed
      if (simulated_clock_pub_) {
        rosgraph_msgs::msg::Clock clock_msg;
        clock_msg.clock = rclcpp::Time(msg->recv_timestamp, RCL_ROS_TIME);
        simulated_clock_pub_->publish(clock_msg);
      }

      // Drain the input queues. Replaces ROS1's reliance on a ros::Timer
      // firing inside spinOnce(); see the note in rosbag_processor.h.
      {
        ProfilerZoneScopedN("rosbagFlushQueues");
        for (const auto& flusher : queue_flushers_) {
          flusher();
        }
      }

      // Process node callbacks (publishers, timers, /clock delivery, ...)
      {
        ProfilerZoneScopedN("rosbagSpinSome");
        rclcpp::spin_some(node_.get_node_base_interface());
      }
    }
  }

  // The bag is exhausted, but the last batch of messages has not been drained
  // yet. ROS1 left this to the retry timer during the post-processing spin;
  // here it is explicit, so no pointcloud is silently dropped at the end.
  for (const auto& flusher : queue_flushers_) {
    flusher();
  }

  return true;
}
}  // namespace wavemap
