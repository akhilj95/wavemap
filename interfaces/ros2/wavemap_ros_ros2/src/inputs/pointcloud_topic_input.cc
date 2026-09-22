#include "wavemap_ros_ros2/inputs/pointcloud_topic_input.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <glog/logging.h>
#include <geometry_msgs/msg/point32.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <wavemap/core/utils/profile/profiler_interface.h>

#include "wavemap_ros_conversions_ros2/time_conversions.h"
#include "wavemap_ros_ros2/utils/pointcloud_msg_builder.h"
#include "wavemap_ros_ros2/utils/private_name.h"

namespace wavemap {
DECLARE_CONFIG_MEMBERS(PointcloudTopicInputConfig,
                      (topic_name)
                      (topic_type)
                      (topic_queue_length)
                      (measurement_integrator_names)
                      (processing_retry_period)
                      (max_wait_for_pose)
                      (sensor_frame_id)
                      (time_offset)
                      (undistort_motion)
                      (num_undistortion_interpolation_intervals_per_cloud)
                      (projected_range_image_topic_name)
                      (undistorted_pointcloud_topic_name));

bool PointcloudTopicInputConfig::isValid(bool verbose) const {
  bool all_valid = true;

  all_valid &= IS_PARAM_NE(topic_name, "", verbose);
  all_valid &= IS_PARAM_TRUE(topic_type.isValid(), verbose);
  all_valid &= IS_PARAM_GT(topic_queue_length, 0, verbose);
  all_valid &= IS_PARAM_FALSE(measurement_integrator_names.empty(), verbose);
  all_valid &= IS_PARAM_GT(processing_retry_period, 0.f, verbose);
  all_valid &= IS_PARAM_GE(max_wait_for_pose, 0.f, verbose);
  all_valid &= IS_PARAM_GT(num_undistortion_interpolation_intervals_per_cloud,
                           0, verbose);

  return all_valid;
}

PointcloudTopicInput::PointcloudTopicInput(
    const PointcloudTopicInputConfig& config,
    std::shared_ptr<Pipeline> pipeline,
    std::shared_ptr<TfTransformer> transformer, std::string world_frame,
    rclcpp::Node& node)
    : RosInputBase(config, std::move(pipeline), transformer,
                   std::move(world_frame), node),
      config_(config.checkValid()),
      pointcloud_undistorter_(
          transformer,
          config_.num_undistortion_interpolation_intervals_per_cloud) {
  // Subscribe to the pointcloud input.
  // NOTE: SensorDataQoS (best effort) rather than the default Reliable. Most
  //       ROS2 sensor drivers publish best effort, and a Reliable subscriber
  //       silently never connects to one -- see claude/issues.md. The history
  //       depth comes from topic_queue_length, the same config field ROS1
  //       used as its subscriber queue length.
  registerCallback(config_.topic_type, [this, &node](auto callback_ptr) {
    using MessageT = decltype(detail::pointcloudCallbackMessageType(
        callback_ptr));
    pointcloud_sub_ = node.create_subscription<MessageT>(
        config_.topic_name,
        rclcpp::SensorDataQoS().keep_last(config_.topic_queue_length),
        [this, callback_ptr](const typename MessageT::ConstSharedPtr msg) {
          std::invoke(callback_ptr, this, *msg);
        });
  });

  // Advertise the range image and undistorted pointcloud publishers if enabled
  if (!config_.projected_range_image_topic_name.empty()) {
    projected_range_image_pub_ = image_transport::create_publisher(
        &node, privateName(config_.projected_range_image_topic_name));
  }
  if (!config_.undistorted_pointcloud_topic_name.empty()) {
    undistorted_pointcloud_pub_ =
        node.create_publisher<sensor_msgs::msg::PointCloud2>(
            privateName(config_.undistorted_pointcloud_topic_name),
            rclcpp::QoS(config_.topic_queue_length));
  }
}

void PointcloudTopicInput::callback(
    const sensor_msgs::msg::PointCloud2& pointcloud_msg) {
  ProfilerZoneScoped;
  // Skip empty clouds
  const size_t num_points = pointcloud_msg.height * pointcloud_msg.width;
  if (num_points == 0) {
    LOG(WARNING) << "Skipping empty pointcloud with timestamp "
                 << rclcpp::Time(pointcloud_msg.header.stamp, RCL_ROS_TIME)
                        .nanoseconds()
                 << ".";
    return;
  }

  // Get the index of the x field, and assert that the y and z fields follow
  auto x_field_iter = std::find_if(
      pointcloud_msg.fields.cbegin(), pointcloud_msg.fields.cend(),
      [](const sensor_msgs::msg::PointField& field) {
        return field.name == "x";
      });
  if (x_field_iter == pointcloud_msg.fields.end()) {
    LOG(WARNING) << "Received pointcloud with missing field x";
    return;
  } else if ((++x_field_iter)->name != "y") {
    LOG(WARNING) << "Received pointcloud with missing or out-of-order field y";
    return;
  } else if ((++x_field_iter)->name != "z") {
    LOG(WARNING) << "Received pointcloud with missing or out-of-order field z";
    return;
  }

  // Convert to our generic stamped pointcloud format.
  // NOTE: The header stamp is wrapped with an explicit RCL_ROS_TIME. In ROS1,
  //       header.stamp *was* a ros::Time; in ROS2 it is a
  //       builtin_interfaces::msg::Time, and rclcpp::Time's default clock
  //       type is RCL_SYSTEM_TIME, which would later throw when compared
  //       against the RCL_ROS_TIME stamps we reconstruct from nanoseconds.
  const uint64_t stamp_nsec = convert::rosTimeToNanoSeconds(
      rclcpp::Time(pointcloud_msg.header.stamp, RCL_ROS_TIME) +
      rclcpp::Duration::from_seconds(config_.time_offset));
  std::string sensor_frame_id = config_.sensor_frame_id.empty()
                                    ? pointcloud_msg.header.frame_id
                                    : config_.sensor_frame_id;
  undistortion::StampedPointcloud stamped_pointcloud{
      stamp_nsec, std::move(sensor_frame_id), num_points};

  // Load the points with time information if undistortion is enabled
  bool loaded = false;
  sensor_msgs::PointCloud2ConstIterator<float> pos_it(pointcloud_msg, "x");
  if (config_.undistort_motion) {
    // NOTE: Livox pointclouds are not handled here but in their own callback.
    switch (config_.topic_type) {
      case PointcloudTopicType::kOuster:
        if (hasField(pointcloud_msg, "t")) {
          sensor_msgs::PointCloud2ConstIterator<uint32_t> t_it(pointcloud_msg,
                                                               "t");
          for (; pos_it != pos_it.end(); ++pos_it, ++t_it) {
            stamped_pointcloud.emplace(pos_it[0], pos_it[1], pos_it[2], *t_it);
          }
          loaded = true;
        } else {
          LOG(WARNING) << "Pointcloud topic type is set to \""
                       << config_.topic_type.toStr()
                       << "\", but message has no time field \"t\". Will not "
                          "be undistorted.";
        }
        break;
      default:
        LOG(WARNING)
            << "Pointcloud undistortion is enabled, but not yet supported for "
               "topic type \""
            << config_.topic_type.toStr() << "\". Will not be undistorted.";
    }
  }

  // If undistortion is disabled or loading failed, only load positions
  if (!loaded) {
    for (; pos_it != pos_it.end(); ++pos_it) {
      stamped_pointcloud.emplace(pos_it[0], pos_it[1], pos_it[2], 0);
    }
  }

  // Add it to the integration queue
  pointcloud_queue_.emplace(std::move(stamped_pointcloud));
}

#ifdef LIVOX_AVAILABLE
void PointcloudTopicInput::callback(
    const livox_ros_driver2::msg::CustomMsg& pointcloud_msg) {
  ProfilerZoneScoped;
  // Skip empty clouds
  if (pointcloud_msg.points.empty()) {
    LOG(WARNING) << "Skipping empty pointcloud with timestamp "
                 << rclcpp::Time(pointcloud_msg.header.stamp, RCL_ROS_TIME)
                        .nanoseconds()
                 << ".";
    return;
  }

  // Convert to our generic stamped pointcloud format
  const uint64_t stamp_nsec =
      pointcloud_msg.timebase +
      static_cast<int32_t>(config_.time_offset * 1000000000.0);
  std::string sensor_frame_id = config_.sensor_frame_id.empty()
                                    ? pointcloud_msg.header.frame_id
                                    : config_.sensor_frame_id;
  undistortion::StampedPointcloud stamped_pointcloud{
      stamp_nsec, std::move(sensor_frame_id), pointcloud_msg.points.size()};
  for (const auto& point : pointcloud_msg.points) {
    stamped_pointcloud.emplace(point.x, point.y, point.z, point.offset_time);
  }

  // Add it to the integration queue
  pointcloud_queue_.emplace(std::move(stamped_pointcloud));
}
#endif

void PointcloudTopicInput::processQueue() {
  ProfilerZoneScoped;
  while (!pointcloud_queue_.empty()) {
    auto& oldest_msg = pointcloud_queue_.front();

    // Drop messages if they're older than max_wait_for_pose
    if (config_.max_wait_for_pose <
        convert::nanoSecondsToSeconds(pointcloud_queue_.back().getEndTime() -
                                      oldest_msg.getStartTime())) {
      LOG(WARNING) << "Max waiting time of " << config_.max_wait_for_pose
                   << "s exceeded for pointcloud with frame \""
                   << oldest_msg.getSensorFrame() << "\" and time interval ["
                   << oldest_msg.getStartTime() << ", "
                   << oldest_msg.getEndTime() << "] vs newest cloud end time "
                   << pointcloud_queue_.back().getEndTime()
                   << ". Dropping cloud.";
      pointcloud_queue_.pop();
      continue;
    }

    // Undistort the pointcloud if appropriate
    PosedPointcloud<> posed_pointcloud;
    if (config_.undistort_motion) {
      const auto undistortion_result =
          pointcloud_undistorter_.undistortPointcloud(
              oldest_msg, posed_pointcloud, world_frame_);
      if (undistortion_result != PointcloudUndistorter::Result::kSuccess) {
        const uint64_t start_time = oldest_msg.getStartTime();
        const uint64_t end_time = oldest_msg.getEndTime();
        switch (undistortion_result) {
          case PointcloudUndistorter::Result::kEndTimeNotInTfBuffer:
            // Try to get this pointcloud's pose again at the next iteration
            return;
          case PointcloudUndistorter::Result::kStartTimeNotInTfBuffer:
            LOG(WARNING)
                << "Pointcloud end pose is available but start pose at time "
                << start_time << " is not (or no longer). Skipping pointcloud.";
            break;
          case PointcloudUndistorter::Result::kIntermediateTimeNotInTfBuffer:
            LOG(WARNING)
                << "Could not buffer all transforms for pointcloud spanning "
                   "time interval ["
                << start_time << ", " << end_time
                << "]. This should never happen. Skipping pointcloud.";
            break;
          default:
            LOG(WARNING) << "Unknown pointcloud undistortion error.";
        }

        pointcloud_queue_.pop();
        continue;
      }
    } else {
      // Get the pointcloud's pose
      const auto T_W_C = transformer_->lookupTransform(
          world_frame_, oldest_msg.getSensorFrame(),
          convert::nanoSecondsToRosTime(oldest_msg.getTimeBase()));
      if (!T_W_C) {
        // Try to get this pointcloud's pose again at the next iteration
        return;
      }

      // Convert to a posed pointcloud
      posed_pointcloud = PosedPointcloud<>(*T_W_C);
      posed_pointcloud.resize(oldest_msg.getPoints().size());
      for (unsigned point_idx = 0; point_idx < oldest_msg.getPoints().size();
           ++point_idx) {
        posed_pointcloud[point_idx] =
            oldest_msg.getPoints()[point_idx].position;
      }
    }

    // Integrate the pointcloud
    VLOG(1) << "Inserting pointcloud with " << posed_pointcloud.size()
            << " points. Remaining pointclouds in queue: "
            << pointcloud_queue_.size() - 1 << ".";
    integration_timer_.start();
    pipeline_->runPipeline(config_.measurement_integrator_names,
                           posed_pointcloud);
    integration_timer_.stop();
    VLOG(1) << "Integrated new pointcloud in "
            << integration_timer_.getLastEpisodeDuration()
            << "s. Total integration time: "
            << integration_timer_.getTotalDuration() << "s.";

    // Publish debugging visualizations
    publishProjectedRangeImageIfEnabled(
        convert::nanoSecondsToRosTime(oldest_msg.getMedianTime()),
        posed_pointcloud);
    publishUndistortedPointcloudIfEnabled(
        convert::nanoSecondsToRosTime(oldest_msg.getMedianTime()),
        posed_pointcloud);
    ProfilerFrameMarkNamed("Pointcloud");

    // Remove the pointcloud from the queue
    pointcloud_queue_.pop();
  }
}

bool PointcloudTopicInput::hasField(const sensor_msgs::msg::PointCloud2& msg,
                                    const std::string& field_name) {
  return std::any_of(msg.fields.cbegin(), msg.fields.cend(),
                     [&field_name = std::as_const(field_name)](
                         const sensor_msgs::msg::PointField& field) {
                       return field.name == field_name;
                     });
}

void PointcloudTopicInput::publishProjectedRangeImageIfEnabled(
    const rclcpp::Time& /*stamp*/,
    const PosedPointcloud<>& /*posed_pointcloud*/) {
  ProfilerZoneScoped;
  if (config_.projected_range_image_topic_name.empty() ||
      projected_range_image_pub_.getNumSubscribers() <= 0) {
    return;
  }

  // TODO(victorr): Reimplement this
  // NOTE: Carried over from ROS1 as-is, including the fact that it publishes
  //       nothing. The topic is still advertised so that the set of topics a
  //       given config produces matches ROS1's exactly.
}

void PointcloudTopicInput::publishUndistortedPointcloudIfEnabled(
    const rclcpp::Time& stamp,
    const PosedPointcloud<>& undistorted_pointcloud) {
  ProfilerZoneScoped;
  if (config_.undistorted_pointcloud_topic_name.empty() ||
      !undistorted_pointcloud_pub_ ||
      undistorted_pointcloud_pub_->get_subscription_count() <= 0) {
    return;
  }

  std_msgs::msg::Header header;
  header.stamp = stamp;
  header.frame_id = world_frame_;

  std::vector<geometry_msgs::msg::Point32> points;
  points.reserve(undistorted_pointcloud.size());
  for (const auto& point : undistorted_pointcloud.getPointsGlobal()) {
    auto& point_msg = points.emplace_back();
    point_msg.x = point.x();
    point_msg.y = point.y();
    point_msg.z = point.z();
  }

  undistorted_pointcloud_pub_->publish(toPointCloud2Msg(points, header));
}
}  // namespace wavemap
