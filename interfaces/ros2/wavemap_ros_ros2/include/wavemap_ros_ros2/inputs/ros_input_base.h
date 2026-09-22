#ifndef WAVEMAP_ROS_ROS2_INPUTS_ROS_INPUT_BASE_H_
#define WAVEMAP_ROS_ROS2_INPUTS_ROS_INPUT_BASE_H_

#include <memory>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <wavemap/core/config/config_base.h>
#include <wavemap/core/config/string_list.h>
#include <wavemap/pipeline/pipeline.h>

#include "wavemap_ros_ros2/utils/tf_transformer.h"

namespace wavemap {
struct RosInputType : public TypeSelector<RosInputType> {
  using TypeSelector<RosInputType>::TypeSelector;

  enum Id : TypeId { kPointcloudTopic, kDepthImageTopic };

  static constexpr std::array names = {"pointcloud_topic", "depth_image_topic"};
};

struct RosInputBaseConfig
    : public ConfigBase<RosInputBaseConfig, 4, StringList> {
  std::string topic_name = "scan";
  int topic_queue_length = 10;

  StringList measurement_integrator_names;

  Seconds<FloatingPoint> processing_retry_period = 0.05f;

  static MemberMap memberMap;

  // Constructors
  RosInputBaseConfig() = default;
  RosInputBaseConfig(std::string topic_name, int topic_queue_length,
                     StringList measurement_integrator_names,
                     FloatingPoint processing_retry_period)
      : topic_name(std::move(topic_name)),
        topic_queue_length(topic_queue_length),
        measurement_integrator_names(std::move(measurement_integrator_names)),
        processing_retry_period(processing_retry_period) {}

  bool isValid(bool verbose) const override;
};

class RosInputBase {
 public:
  RosInputBase(const RosInputBaseConfig& config,
               std::shared_ptr<Pipeline> pipeline,
               std::shared_ptr<TfTransformer> transformer,
               std::string world_frame, rclcpp::Node& node);
  virtual ~RosInputBase() = default;

  virtual RosInputType getType() const = 0;
  const std::string& getTopicName() { return config_.topic_name; }

  // Drain the input's queue immediately, rather than waiting for the retry
  // timer to fire.
  //
  // ROS1 drove processQueue() exclusively from a ros::Timer, which fires
  // during ros::spinOnce(). The offline rosbag processor therefore depended
  // on the simulated /clock it published itself being routed back to its own
  // node before any queue was drained. Offline, that round-trip buys nothing
  // and costs determinism, so the ROS2 processor calls this directly instead.
  // The live node still uses the timer, exactly as ROS1 does.
  void processQueueNow() { processQueue(); }

 protected:
  const RosInputBaseConfig config_;

  std::shared_ptr<Pipeline> pipeline_;

  const std::shared_ptr<TfTransformer> transformer_;
  const std::string world_frame_;

  virtual void processQueue() = 0;
  rclcpp::TimerBase::SharedPtr queue_processing_retry_timer_;
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_ROS2_INPUTS_ROS_INPUT_BASE_H_
