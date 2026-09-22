#include "wavemap_ros_ros2/ros_server.h"

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <wavemap/core/map/map_factory.h>
#include <wavemap/io/file_conversions.h>
#include <wavemap/pipeline/map_operations/map_operation_factory.h>

#include "wavemap_ros_conversions_ros2/config_conversions.h"
#include "wavemap_ros_ros2/inputs/ros_input_factory.h"
#include "wavemap_ros_ros2/map_operations/map_ros_operation_factory.h"
#include "wavemap_ros_ros2/utils/private_name.h"

namespace wavemap {
namespace {
// Pull one top-level section out of the config tree.
//
// ROS1 fetched each section from the parameter server separately and warned,
// then returned an empty map/array, when it was missing
// (interfaces/ros1/wavemap_ros_conversions/src/config_conversions.cc:6-37).
// The sections now come from one parsed file instead, but the
// missing-section behaviour is kept identical so that a partial config
// behaves the same on both interfaces.
param::Value getSection(const param::Value& params, const std::string& name) {
  if (const auto child = params.getChild(name); child) {
    return child.value();
  }
  LOG(WARNING) << "Could not load config section \"" << name << "\".";
  return param::Value{param::Map{}};
}

param::Array getSectionAsArray(const param::Value& params,
                               const std::string& name) {
  if (const auto child = params.getChild(name); child) {
    if (const auto array = child->as<param::Array>(); array) {
      return array.value();
    }
    LOG(WARNING) << "Config section \"" << name << "\" is not a list.";
    return {};
  }
  LOG(WARNING) << "Could not load config section \"" << name << "\".";
  return {};
}

param::Map getSectionAsMap(const param::Value& params,
                           const std::string& name) {
  if (const auto child = params.getChild(name); child) {
    if (const auto map = child->as<param::Map>(); map) {
      return map.value();
    }
    LOG(WARNING) << "Config section \"" << name << "\" is not a map.";
    return {};
  }
  LOG(WARNING) << "Could not load config section \"" << name << "\".";
  return {};
}
}  // namespace

DECLARE_CONFIG_MEMBERS(RosServerConfig,
                      (world_frame)
                      (num_threads)
                      (logging_level)
                      (allow_reset_map_service));

bool RosServerConfig::isValid(bool verbose) const {
  bool all_valid = true;

  all_valid &= IS_PARAM_NE(world_frame, "", verbose);
  all_valid &= IS_PARAM_GT(num_threads, 0, verbose);
  all_valid &= IS_PARAM_TRUE(logging_level.isValid(), verbose);

  return all_valid;
}

param::Value RosServer::loadParams(rclcpp::Node& node) {
  const std::string config_file =
      node.has_parameter(kConfigFileParamName)
          ? node.get_parameter(kConfigFileParamName).as_string()
          : node.declare_parameter<std::string>(kConfigFileParamName, "");
  if (config_file.empty()) {
    const std::string message =
        std::string{"ROS parameter \""} + kConfigFileParamName +
        "\" is not set. It must point at wavemap's YAML config file.";
    LOG(ERROR) << message;
    throw std::runtime_error(message);
  }

  const auto params = param::convert::yamlFileToParams(config_file);
  if (!params) {
    // NOTE: yamlFileToParams already logged why.
    throw std::runtime_error("Could not load wavemap config file: " +
                             config_file);
  }
  LOG(INFO) << "Loaded wavemap config from " << config_file;
  return params.value();
}

// NOTE: If RosServerConfig::from(...) fails, accessing its value will throw
//       an exception and end the program.
RosServer::RosServer(rclcpp::Node& node,
                     std::shared_ptr<TfTransformer> transformer)
    : RosServer(node, loadParams(node), std::move(transformer)) {}

RosServer::RosServer(rclcpp::Node& node, const param::Value& params,
                     std::shared_ptr<TfTransformer> transformer)
    : RosServer(node, params,
                RosServerConfig::from(getSection(params, "general")).value(),
                std::move(transformer)) {}

RosServer::RosServer(rclcpp::Node& node, const param::Value& params,
                     const RosServerConfig& config,
                     std::shared_ptr<TfTransformer> transformer)
    : config_(config.checkValid()),
      transformer_(transformer ? std::move(transformer)
                               : std::make_shared<TfTransformer>(node)) {
  // Set the logging level for wavemap's C++ library (uses glog) and ROS
  config_.logging_level.applyToGlog();
  config_.logging_level.applyToRosConsole(node.get_logger().get_name());

  // Setup data structure
  const auto data_structure_params = getSection(params, "map");
  occupancy_map_ =
      MapFactory::create(data_structure_params, MapType::kHashedBlocks);
  CHECK_NOTNULL(occupancy_map_);

  // Setup thread pool
  LOG(INFO) << "Creating thread pool with " << config_.num_threads
            << " threads.";
  thread_pool_ = std::make_shared<ThreadPool>(config_.num_threads);
  CHECK_NOTNULL(thread_pool_);

  // Setup the pipeline
  pipeline_ = std::make_shared<Pipeline>(occupancy_map_, thread_pool_);
  CHECK_NOTNULL(pipeline_);

  // Add map operations to pipeline
  const param::Array map_operation_param_array =
      getSectionAsArray(params, "map_operations");
  for (const auto& operation_params : map_operation_param_array) {
    addOperation(operation_params, node);
  }

  // Add measurement integrators to pipeline
  const param::Map measurement_integrator_param_map =
      getSectionAsMap(params, "measurement_integrators");
  for (const auto& [integrator_name, integrator_params] :
       measurement_integrator_param_map) {
    pipeline_->addIntegrator(integrator_name, integrator_params);
  }

  // Setup measurement inputs
  const param::Array input_param_array = getSectionAsArray(params, "inputs");
  for (const auto& integrator_params : input_param_array) {
    addInput(integrator_params, node);
  }

  // Connect to ROS
  advertiseServices(node);
}

void RosServer::clear() {
  clearInputs();
  if (pipeline_) {
    pipeline_->clear();
  }
  if (occupancy_map_) {
    occupancy_map_->clear();
  }
}

RosInputBase* RosServer::addInput(const param::Value& integrator_params,
                                  rclcpp::Node& node) {
  auto input = RosInputFactory::create(integrator_params, pipeline_,
                                       transformer_, config_.world_frame, node);
  return addInput(std::move(input));
}

RosInputBase* RosServer::addInput(std::unique_ptr<RosInputBase> input) {
  if (input) {
    return inputs_.emplace_back(std::move(input)).get();
  }

  LOG(WARNING) << "Ignoring request to add input. Input is null pointer.";
  return nullptr;
}

MapOperationBase* RosServer::addOperation(const param::Value& operation_params,
                                          rclcpp::Node& node) {
  // Read the operation type name from params
  const auto type_name = param::getTypeStr(operation_params);
  if (!type_name) {
    // No type name was defined
    // NOTE: A message explaining the failure is already printed by getTypeStr.
    LOG(WARNING) << "Could not add operation. No operation type specified. "
                    "Please set it by adding a param with key \""
                 << param::kTypeSelectorKey << "\".";
    return nullptr;
  }

  if (const auto type = MapRosOperationType{type_name.value()};
      type.isValid()) {
    auto operation = MapRosOperationFactory::create(
        type, operation_params, occupancy_map_, thread_pool_, transformer_,
        config_.world_frame, node);
    // NOTE: The factory returns nullptr for ROS operations that are not yet
    //       ported. Pipeline::addOperation would dereference it, so those are
    //       dropped here instead. The factory has already said which and why.
    if (!operation) {
      return nullptr;
    }
    return pipeline_->addOperation(std::move(operation));
  }

  if (const auto type = MapOperationType{type_name.value()}; type.isValid()) {
    auto operation =
        MapOperationFactory::create(type, operation_params, occupancy_map_);
    return pipeline_->addOperation(std::move(operation));
  }

  LOG(WARNING) << "Value of type name param \"" << param::kTypeSelectorKey
               << "\": \"" << type_name.value()
               << "\" does not match a known operation type name. Supported "
                  "type names are ["
               << print::sequence(MapRosOperationType::names) << ", "
               << print::sequence(MapOperationType::names) << "].";
  return nullptr;
}

bool RosServer::saveMap(const std::filesystem::path& file_path) const {
  if (occupancy_map_) {
    occupancy_map_->threshold();
    return io::mapToFile(*occupancy_map_, file_path);
  } else {
    LOG(ERROR) << "Could not save map because it has not yet been allocated.";
  }
  return false;
}

bool RosServer::loadMap(const std::filesystem::path& file_path) {
  return io::fileToMap(file_path, occupancy_map_);
}

void RosServer::advertiseServices(rclcpp::Node& node) {
  using Trigger = std_srvs::srv::Trigger;
  using FilePath = wavemap_msgs_ros2::srv::FilePath;

  // NOTE: The "~/" prefixes are load-bearing. ROS1 advertised these through
  //       nh_private, whose namespace is the node name; ROS2 resolves a
  //       relative name against the node's namespace instead. Without the
  //       tilde these would land at /reset_map rather than
  //       /wavemap/reset_map. See claude/issues.md.
  reset_map_srv_ = node.create_service<Trigger>(
      privateName("reset_map"),
      [this](const std::shared_ptr<Trigger::Request> /*request*/,
             std::shared_ptr<Trigger::Response> response) {
        response->success = false;
        if (config_.allow_reset_map_service) {
          if (occupancy_map_) {
            occupancy_map_->clear();
          }
          LOG(INFO) << "Map reset request was successfully executed.";
          response->success = true;
        } else {
          response->message =
              "Map resetting is forbidden. To change this, set ROS param \"" +
              NAMEOF(config_.allow_reset_map_service) + "\" to true.";
          LOG(INFO) << "Received map reset request but ROS param \""
                    << NAMEOF(config_.allow_reset_map_service)
                    << "\" is set to false. Ignoring request.";
        }
      });

  save_map_srv_ = node.create_service<FilePath>(
      privateName("save_map"),
      [this](const std::shared_ptr<FilePath::Request> request,
             std::shared_ptr<FilePath::Response> response) {
        response->success = saveMap(request->file_path);
      });

  load_map_srv_ = node.create_service<FilePath>(
      privateName("load_map"),
      [this](const std::shared_ptr<FilePath::Request> request,
             std::shared_ptr<FilePath::Response> response) {
        response->success = loadMap(request->file_path);
      });
}
}  // namespace wavemap
