#ifndef WAVEMAP_ROS_ROS2_ROS_SERVER_H_
#define WAVEMAP_ROS_ROS2_ROS_SERVER_H_

#include <algorithm>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <glog/logging.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <wavemap/core/common.h>
#include <wavemap/core/config/config_base.h>
#include <wavemap/core/indexing/index_hashes.h>
#include <wavemap/core/integrator/integrator_base.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap/core/utils/thread_pool.h>
#include <wavemap/pipeline/pipeline.h>
#include <wavemap_msgs_ros2/srv/file_path.hpp>

#include "wavemap_ros_ros2/inputs/ros_input_base.h"
#include "wavemap_ros_ros2/utils/ros_logging_level.h"
#include "wavemap_ros_ros2/utils/tf_transformer.h"

namespace wavemap {
/**
 * Config struct for wavemap's ROS server.
 */
struct RosServerConfig : ConfigBase<RosServerConfig, 4, RosLoggingLevel> {
  //! Name of the coordinate frame in which to store the map.
  //! Will be used as the frame_id for ROS TF lookups.
  std::string world_frame = "odom";
  //! Minimum severity level for messages to be logged.
  RosLoggingLevel logging_level = RosLoggingLevel::kInfo;
  //! Maximum number of threads to use.
  //! Defaults to the number of threads supported by the CPU.
  int num_threads =
      std::max(1, static_cast<int>(std::thread::hardware_concurrency()));
  //! Whether or not to allow resetting the map through the reset_map service.
  bool allow_reset_map_service = false;

  static MemberMap memberMap;

  bool isValid(bool verbose) const override;
};

class RosServer {
 public:
  // The name of the ROS2 parameter holding the path to wavemap's YAML config.
  //
  // This single string parameter replaces ROS1's <rosparam file="..."> tag.
  // ROS2's parameter system cannot represent a list of heterogeneous dicts,
  // which `map_operations:` and `inputs:` both are, so the file is read
  // directly instead of being pushed through the parameter server. The YAML
  // schema is unchanged: a ROS1 user's config file works as-is. See
  // claude/decisions.md section 2.
  static constexpr const char* kConfigFileParamName = "general.config_file";

  // Reads kConfigFileParamName and loads the config file it points at.
  // Throws std::runtime_error if the file is missing or unparseable, matching
  // ROS1, where a failed RosServerConfig::from(...) threw out of the
  // constructor and ended the program.
  //
  // `transformer` is optional. Passing nullptr (the default) makes the server
  // build its own TfTransformer, which subscribes to /tf and /tf_static like
  // ROS1's did. The offline rosbag processor passes a listener-free
  // transformer instead, so that a stray /tf publisher elsewhere on the
  // network cannot perturb an otherwise deterministic replay.
  explicit RosServer(rclcpp::Node& node,
                     std::shared_ptr<TfTransformer> transformer = nullptr);
  RosServer(rclcpp::Node& node, const param::Value& params,
            std::shared_ptr<TfTransformer> transformer = nullptr);
  RosServer(rclcpp::Node& node, const param::Value& params,
            const RosServerConfig& config,
            std::shared_ptr<TfTransformer> transformer = nullptr);

  void clear();

  MapBase::Ptr getMap() { return occupancy_map_; }
  MapBase::ConstPtr getMap() const { return occupancy_map_; }

  Pipeline& getPipeline() { return *pipeline_; }
  const Pipeline& getPipeline() const { return *pipeline_; }

  std::shared_ptr<TfTransformer> getTransformer() { return transformer_; }

  MapOperationBase* addOperation(const param::Value& operation_params,
                                 rclcpp::Node& node);

  RosInputBase* addInput(const param::Value& integrator_params,
                         rclcpp::Node& node);
  RosInputBase* addInput(std::unique_ptr<RosInputBase> input);
  const std::vector<std::unique_ptr<RosInputBase>>& getInputs() {
    return inputs_;
  }
  void clearInputs() { inputs_.clear(); }

  bool saveMap(const std::filesystem::path& file_path) const;
  bool loadMap(const std::filesystem::path& file_path);

  // Load wavemap's YAML config from the path in kConfigFileParamName.
  static param::Value loadParams(rclcpp::Node& node);

 private:
  const RosServerConfig config_;

  // Map data structure
  MapBase::Ptr occupancy_map_;

  // Threadpool shared among all input handlers and operations
  std::shared_ptr<ThreadPool> thread_pool_;

  // Map management pipeline
  std::shared_ptr<Pipeline> pipeline_;

  // Measurement and pose inputs
  std::vector<std::unique_ptr<RosInputBase>> inputs_;
  std::shared_ptr<TfTransformer> transformer_;

  // ROS services
  void advertiseServices(rclcpp::Node& node);
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_map_srv_;
  rclcpp::Service<wavemap_msgs_ros2::srv::FilePath>::SharedPtr save_map_srv_;
  rclcpp::Service<wavemap_msgs_ros2::srv::FilePath>::SharedPtr load_map_srv_;
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_ROS2_ROS_SERVER_H_
