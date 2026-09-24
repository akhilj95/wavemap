#ifndef WAVEMAP_ROS_ROS2_INPUTS_DEPTH_IMAGE_TOPIC_INPUT_H_
#define WAVEMAP_ROS_ROS2_INPUTS_DEPTH_IMAGE_TOPIC_INPUT_H_

#include <memory>
#include <queue>
#include <string>

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <wavemap/core/config/string_list.h>
#include <wavemap/core/data_structure/image.h>
#include <wavemap/core/integrator/projective/projective_integrator.h>
#include <wavemap/core/utils/time/stopwatch.h>

#include "wavemap_ros_ros2/inputs/ros_input_base.h"

namespace wavemap {
/**
 * Config struct for the depth image input handler.
 */
struct DepthImageTopicInputConfig
    : public ConfigBase<DepthImageTopicInputConfig, 10, StringList> {
  //! Name of the ROS topic to subscribe to.
  std::string topic_name;
  //! Queue length to use when subscribing to the ROS topic.
  int topic_queue_length = 10;

  //! Name(s) of the measurement integrator(s) used to integrate the
  //! measurements into the map.
  StringList measurement_integrator_names;

  //! Time period used to control the rate at which to retry getting the sensor
  //! pose when ROS TF lookups fail.
  Seconds<FloatingPoint> processing_retry_period = 0.05f;
  //! Maximum amount of time to wait for the sensor pose to become available
  //! when ROS TF lookups fail.
  Seconds<FloatingPoint> max_wait_for_pose = 1.f;

  //! The frame_id to use to look up the sensor pose using ROS TFs.
  //! Note that setting this is optional, when left blank the header.frame_id of
  //! the measurement's msg is used.
  std::string sensor_frame_id;
  //! Name of the image_transport transport to use when subscribing to the
  //! depth image topic, e.g. 'raw' or 'compressedDepth'. Defaults to 'raw'.
  std::string image_transport_hints = "raw";
  //! Scale factor used to convert the depth image's values to meters.
  FloatingPoint depth_scale_factor = 1.f;
  //! Time offset to apply to the header.stamp of the measurement's msg when
  //! looking up its pose using ROS TFs. Can be used when the time offset is
  //! known (e.g. through calibration) but not corrected by the sensor's driver.
  Seconds<FloatingPoint> time_offset = 0.f;

  //! Name of the topic on which to publish the depth images as pointclouds.
  //! Useful to share the pointclouds with other ROS nodes and for debugging.
  //! Disabled if not set.
  std::string projected_pointcloud_topic_name;

  static MemberMap memberMap;

  // Conversion to InputBase config
  operator RosInputBaseConfig() const {  // NOLINT
    return {topic_name, topic_queue_length, measurement_integrator_names,
            processing_retry_period};
  }

  bool isValid(bool verbose) const override;
};

class DepthImageTopicInput : public RosInputBase {
 public:
  DepthImageTopicInput(const DepthImageTopicInputConfig& config,
                       std::shared_ptr<Pipeline> pipeline,
                       std::shared_ptr<TfTransformer> transformer,
                       std::string world_frame, rclcpp::Node& node);

  RosInputType getType() const override {
    return RosInputType::kDepthImageTopic;
  }

  void callback(const sensor_msgs::msg::Image& depth_image_msg) {
    depth_image_queue_.emplace(depth_image_msg);
  }

 private:
  const DepthImageTopicInputConfig config_;

  Stopwatch integration_timer_;

  image_transport::Subscriber depth_image_sub_;
  // NOTE: No mutex, for the same reason as in PointcloudTopicInput: the node
  //       runs under a SingleThreadedExecutor, as ROS1 ran a single-threaded
  //       spinner.
  std::queue<sensor_msgs::msg::Image> depth_image_queue_;
  void processQueue() override;

  void publishProjectedPointcloudIfEnabled(
      const rclcpp::Time& stamp,
      const PosedImage<FloatingPoint>& posed_depth_image);
  static PosedPointcloud<Point3D> project(
      const PosedImage<>& posed_depth_image,
      const ProjectorBase& projection_model);
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      projected_pointcloud_pub_;
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_ROS2_INPUTS_DEPTH_IMAGE_TOPIC_INPUT_H_
