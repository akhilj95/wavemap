#ifndef WAVEMAP2_ROS_INPUTS_ROS_INPUT_BASE_H_
#define WAVEMAP2_ROS_INPUTS_ROS_INPUT_BASE_H_

#include <memory>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <wavemap/core/config/config_base.h>
#include <wavemap/core/config/string_list.h>
#include <wavemap/pipeline/pipeline.h>

#include "wavemap2_ros/utils/tf_transformer.h"

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
  // timer to fire
  // NOTE: Used by the rosbag processor, whose timers would only fire once the
  //       simulated /clock it publishes reached its own node.
  void processQueueNow() { processQueue(); }

 protected:
  const RosInputBaseConfig config_;

  std::shared_ptr<Pipeline> pipeline_;

  const std::shared_ptr<TfTransformer> transformer_;
  const std::string world_frame_;
  const rclcpp::Logger logger_;

  virtual void processQueue() = 0;
  rclcpp::TimerBase::SharedPtr queue_processing_retry_timer_;
};
}  // namespace wavemap

#endif  // WAVEMAP2_ROS_INPUTS_ROS_INPUT_BASE_H_
