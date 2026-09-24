#include "wavemap_ros_ros2/inputs/depth_image_topic_input.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <glog/logging.h>
#include <opencv2/core/eigen.hpp>
#include <wavemap/core/integrator/projective/projective_integrator.h>
#include <wavemap/core/utils/iterate/grid_iterator.h>
#include <wavemap/core/utils/print/eigen.h>
#include <wavemap/core/utils/profile/profiler_interface.h>

#include "wavemap_ros_ros2/utils/private_name.h"

namespace wavemap {
DECLARE_CONFIG_MEMBERS(DepthImageTopicInputConfig,
                      (topic_name)
                      (topic_queue_length)
                      (measurement_integrator_names)
                      (processing_retry_period)
                      (max_wait_for_pose)
                      (sensor_frame_id)
                      (image_transport_hints)
                      (depth_scale_factor)
                      (time_offset)
                      (projected_pointcloud_topic_name));

bool DepthImageTopicInputConfig::isValid(bool verbose) const {
  bool all_valid = true;

  all_valid &= IS_PARAM_NE(topic_name, "", verbose);
  all_valid &= IS_PARAM_GT(topic_queue_length, 0, verbose);
  all_valid &= IS_PARAM_FALSE(measurement_integrator_names.empty(), verbose);
  all_valid &= IS_PARAM_GT(processing_retry_period, 0.f, verbose);
  all_valid &= IS_PARAM_GE(max_wait_for_pose, 0.f, verbose);

  return all_valid;
}

DepthImageTopicInput::DepthImageTopicInput(
    const DepthImageTopicInputConfig& config,
    std::shared_ptr<Pipeline> pipeline,
    std::shared_ptr<TfTransformer> transformer, std::string world_frame,
    rclcpp::Node& node)
    : RosInputBase(config, std::move(pipeline), std::move(transformer),
                   std::move(world_frame), node),
      config_(config.checkValid()) {
  // Subscribe to the depth image input.
  // NOTE: SensorDataQoS (best effort), as for the pointcloud input: a best
  //       effort subscriber connects to both best effort and reliable
  //       publishers, whereas a reliable one silently never connects to a best
  //       effort publisher. See claude/issues.md.
  depth_image_sub_ = image_transport::create_subscription(
      &node, config_.topic_name,
      [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        callback(*msg);
      },
      config_.image_transport_hints,
      rclcpp::SensorDataQoS()
          .keep_last(config_.topic_queue_length)
          .get_rmw_qos_profile());

  // Advertise the projected pointcloud publisher if enabled
  if (!config_.projected_pointcloud_topic_name.empty()) {
    projected_pointcloud_pub_ =
        node.create_publisher<sensor_msgs::msg::PointCloud2>(
            privateName(config_.projected_pointcloud_topic_name),
            rclcpp::QoS(config_.topic_queue_length));
  }
}

void DepthImageTopicInput::processQueue() {
  ProfilerZoneScoped;
  while (!depth_image_queue_.empty()) {
    const sensor_msgs::msg::Image& oldest_msg = depth_image_queue_.front();
    const std::string sensor_frame_id = config_.sensor_frame_id.empty()
                                            ? oldest_msg.header.frame_id
                                            : config_.sensor_frame_id;

    // Get the sensor pose in world frame.
    // NOTE: Header stamps are wrapped with an explicit RCL_ROS_TIME, for the
    //       same reason as in PointcloudTopicInput::callback().
    const rclcpp::Time oldest_stamp(oldest_msg.header.stamp, RCL_ROS_TIME);
    const rclcpp::Time stamp =
        oldest_stamp + rclcpp::Duration::from_seconds(config_.time_offset);
    const auto T_W_C =
        transformer_->lookupTransform(world_frame_, sensor_frame_id, stamp);
    if (!T_W_C) {
      const rclcpp::Time newest_stamp(depth_image_queue_.back().header.stamp,
                                      RCL_ROS_TIME);
      if ((newest_stamp - oldest_stamp).seconds() <
          config_.max_wait_for_pose) {
        // Try to get this depth image's pose again at the next iteration
        return;
      } else {
        LOG(WARNING) << "Waited " << config_.max_wait_for_pose
                     << "s but still could not look up pose for depth image "
                        "with frame \""
                     << sensor_frame_id << "\" in world frame \""
                     << world_frame_ << "\" at timestamp "
                     << stamp.nanoseconds() << "; skipping depth image.";
        depth_image_queue_.pop();
        continue;
      }
    }

    // Convert the depth image to our coordinate convention
    auto cv_image = cv_bridge::toCvCopy(oldest_msg);
    if (!cv_image) {
      return;
    }
    cv::transpose(cv_image->image, cv_image->image);
    cv_image->image.convertTo(cv_image->image, CV_32FC1,
                              config_.depth_scale_factor);

    // Create the posed depth image input
    PosedImage<> posed_depth_image(cv_image->image.rows, cv_image->image.cols);
    cv::cv2eigen<FloatingPoint>(cv_image->image, posed_depth_image.getData());
    posed_depth_image.setPose(*T_W_C);

    // Integrate the depth image
    VLOG(1) << "Inserting depth image with "
            << print::eigen::oneLine(posed_depth_image.getDimensions())
            << " points. Remaining pointclouds in queue: "
            << depth_image_queue_.size() - 1 << ".";
    integration_timer_.start();
    pipeline_->runPipeline(config_.measurement_integrator_names,
                           posed_depth_image);
    integration_timer_.stop();
    VLOG(1) << "Integrated new depth image in "
            << integration_timer_.getLastEpisodeDuration()
            << "s. Total integration time: "
            << integration_timer_.getTotalDuration() << "s.";

    // Publish debugging visualizations
    publishProjectedPointcloudIfEnabled(stamp, posed_depth_image);
    ProfilerFrameMarkNamed("DepthImage");

    // Remove the depth image from the queue
    depth_image_queue_.pop();
  }
}

void DepthImageTopicInput::publishProjectedPointcloudIfEnabled(
    const rclcpp::Time& /*stamp*/,
    const PosedImage<FloatingPoint>& /*posed_depth_image*/) {
  ProfilerZoneScoped;
  if (config_.projected_pointcloud_topic_name.empty() ||
      !projected_pointcloud_pub_ ||
      projected_pointcloud_pub_->get_subscription_count() <= 0) {
    return;
  }

  // TODO(victorr): Reimplement this
  // NOTE: Carried over from ROS1 as-is, including the fact that it publishes
  //       nothing. The topic is still advertised so that the set of topics a
  //       given config produces matches ROS1's exactly.
}

PosedPointcloud<Point3D> DepthImageTopicInput::project(
    const PosedImage<>& posed_depth_image,
    const ProjectorBase& projection_model) {
  ProfilerZoneScoped;
  std::vector<Point3D> pointcloud;
  pointcloud.reserve(posed_depth_image.size());
  for (const Index2D& index :
       Grid<2>(Index2D::Zero(),
               posed_depth_image.getDimensions() - Index2D::Ones())) {
    const Vector2D image_xy = projection_model.indexToImage(index);
    const FloatingPoint image_z = posed_depth_image.at(index);
    const Point3D C_point =
        projection_model.sensorToCartesian(image_xy, image_z);
    pointcloud.emplace_back(C_point);
  }

  return PosedPointcloud<Point3D>{posed_depth_image.getPose(), pointcloud};
}
}  // namespace wavemap
