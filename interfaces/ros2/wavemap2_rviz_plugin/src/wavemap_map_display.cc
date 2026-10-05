#include "wavemap2_rviz_plugin/wavemap_map_display.h"

#include <memory>
#include <string>

#include <OgreSceneNode.h>
#include <QTimer>
#include <qfiledialog.h>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <wavemap/core/utils/profile/profiler_interface.h>
#include <wavemap/io/file_conversions.h>
#include <wavemap2_ros_conversions/map_msg_conversions.h>

#include "wavemap2_rviz_plugin/utils/alert_dialog.h"

namespace wavemap::rviz_plugin {
WavemapMapDisplay::WavemapMapDisplay() {
  // Initialize the property menu
  source_mode_property_.clearOptions();
  for (const auto& name : SourceMode::names) {
    source_mode_property_.addOption(name);
  }
  source_mode_property_.setStringStd(source_mode_.toStr());
}

// After the top-level rviz_common::Display::initialize() does its own setup,
// it calls the subclass's onInitialize() function. This is where we
// instantiate all the workings of the class. We make sure to also
// call our immediate super-class's onInitialize() function, since it
// does important stuff setting up the message filter.
void WavemapMapDisplay::onInitialize() {
  ProfilerZoneScoped;
  MFDClass::onInitialize();
  voxel_visual_ = std::make_unique<VoxelVisual>(
      scene_manager_, context_->getViewManager(), scene_node_,
      &voxel_visual_properties_, map_and_mutex_);
  slice_visual_ = std::make_unique<SliceVisual>(
      scene_manager_, scene_node_, &slice_visual_properties_, map_and_mutex_);
}

// Clear the visuals by deleting their objects.
void WavemapMapDisplay::reset() {
  ProfilerZoneScoped;
  MFDClass::reset();
  voxel_visual_->clear();
  slice_visual_->clear();
}

bool WavemapMapDisplay::hasMap() {
  ProfilerZoneScoped;
  std::scoped_lock lock(map_and_mutex_->mutex);
  return static_cast<bool>(map_and_mutex_->map);
}

void WavemapMapDisplay::clearMap() {
  ProfilerZoneScoped;
  std::scoped_lock lock(map_and_mutex_->mutex);
  if (map_and_mutex_->map) {
    map_and_mutex_->map->clear();
  }
}

bool WavemapMapDisplay::loadMapFromDisk(const std::filesystem::path& filepath) {
  ProfilerZoneScoped;
  std::scoped_lock lock(map_and_mutex_->mutex);
  return io::fileToMap(filepath, map_and_mutex_->map);
}

void WavemapMapDisplay::updateVisuals(bool redraw_all) {
  ProfilerZoneScoped;
  if (!hasMap()) {
    return;
  }
  voxel_visual_->updateMap(redraw_all);
  slice_visual_->update();
}

// This is our callback to handle an incoming message
void WavemapMapDisplay::processMessage(
    wavemap2_msgs::msg::Map::ConstSharedPtr map_msg) {
  ProfilerZoneScoped;
  // Deserialize the octree
  if (!map_msg) {
    RVIZ_COMMON_LOG_WARNING(
        "Ignoring request to process non-existent octree msg (nullptr).");
    return;
  }
  updateMapFromRosMsg(*map_msg);

  // Check that the visuals are initialized before continuing
  if (!voxel_visual_ || !slice_visual_) {
    RVIZ_COMMON_LOG_WARNING("Visuals not initialized yet, skipping message.");
    return;
  }

  // Here we call the rviz_common::FrameManagerIface to get the transform from
  // the fixed frame to the frame in the header of this WavemapOctree message.
  // If it fails, we can't do anything else, so we return.
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(map_msg->header, position,
                                                 orientation)) {
    RVIZ_COMMON_LOG_WARNING_STREAM("Error transforming from frame '"
                                   << map_msg->header.frame_id << "' to frame '"
                                   << qPrintable(fixed_frame_) << "'");
    return;
  }
  voxel_visual_->setFramePosition(position);
  voxel_visual_->setFrameOrientation(orientation);
  slice_visual_->setFramePosition(position);
  slice_visual_->setFrameOrientation(orientation);

  // Update the voxel and slice visual's contents if they exist
  updateVisuals();
}

void WavemapMapDisplay::updateMapFromRosMsg(
    const wavemap2_msgs::msg::Map& map_msg) {
  ProfilerZoneScoped;
  std::scoped_lock lock(map_and_mutex_->mutex);
  if (!convert::rosMsgToMap(map_msg, map_and_mutex_->map)) {
    RVIZ_COMMON_LOG_WARNING("Failed to parse map message.");
  }
}

void WavemapMapDisplay::updateSourceModeCallback() {
  ProfilerZoneScoped;
  // Update the cached source mode value
  const SourceMode old_source_mode = source_mode_;
  source_mode_ = SourceMode(source_mode_property_.getStdString());

  // Show/hide the properties appropriate for mode kFromTopic
  // NOTE: RViz2's QoS settings are children of the topic property, so they
  //       are hidden along with it.
  topic_property_->setHidden(source_mode_ != SourceMode::kFromTopic);
  message_queue_property_->setHidden(source_mode_ != SourceMode::kFromTopic);
  request_whole_map_property_.setHidden(source_mode_ != SourceMode::kFromTopic);
  request_wavemap_server_reset_property_.setHidden(source_mode_ !=
                                                   SourceMode::kFromTopic);

  // Show/hide the properties appropriate for mode kFromFile
  load_map_from_disk_property_.setHidden(source_mode_ != SourceMode::kFromFile);
  load_map_from_disk_property_.resetAllValues();

  // Update the map if the source mode changed
  if (source_mode_ != old_source_mode) {
    // Subscribe to the ROS topic if appropriate
    if (source_mode_ == SourceMode::kFromTopic) {
      updateTopic();
    } else {  // Otherwise, unsubscribe
      unsubscribe();
    }
    // Reset the map and update the visuals
    clearMap();
    updateVisuals(true);
  }
}

template <typename ServiceT>
bool WavemapMapDisplay::connectServiceClient(
    const std::string& map_topic,
    const std::optional<std::string>& service_name,
    typename rclcpp::Client<ServiceT>::SharedPtr& client) {
  if (service_name) {
    // If we managed to resolve the service name,
    // check if it differs from our current connection
    if (!client || client->get_service_name() != service_name.value()) {
      // If so, update it
      client = rviz_ros_node_.lock()->get_raw_node()->create_client<ServiceT>(
          service_name.value());
    }
  } else {
    // If the service name could not be resolved,
    // make sure we don't stay connected to a service that's no longer relevant
    client.reset();

    // Alert the user that the service name could not be resolved
    AlertDialog alert{"Not available",
                      "Could not resolve the wavemap_server's namespace from "
                      "the map topic selected in Rviz. Does \"" +
                          map_topic +
                          "\" point to a wavemap_server's map topic?"};
    alert.exec();
    return false;
  }

  // Check whether the service is available
  if (!client->wait_for_service(kServiceWaitTime)) {
    AlertDialog alert{"Not available",
                      "Could not connect to service:\n\"" +
                          service_name.value() +
                          "\".\nIs the wavemap_server running and is the map "
                          "topic selected in Rviz correct?"};
    alert.exec();
    return false;
  }

  return true;
}

// NOTE: The services are called asynchronously, as RViz2 spins its node on the
//       GUI thread and a blocking call would freeze the interface until the
//       server responds. This also means there is no equivalent of ROS1's
//       "exists but could not be called" failure; a server that stops
//       responding simply leaves the request pending.
void WavemapMapDisplay::requestWavemapServerResetCallback() {
  ProfilerZoneScoped;
  // Resolve name of the service based on the map topic
  const std::string map_topic = topic_property_->getTopicStd();
  const auto service_name = resolveWavemapServerNamespaceFromMapTopic(
      map_topic, kResetWavemapServerService);
  if (!connectServiceClient<std_srvs::srv::Trigger>(
          map_topic, service_name, request_wavemap_server_reset_client_)) {
    return;
  }

  // Call the service
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  request_wavemap_server_reset_client_->async_send_request(
      request,
      [this, service_name = service_name.value()](
          rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        const auto response = future.get();
        if (!response->success) {
          // Alert the user if the call succeeded but the action did not
          showAlertLater("Error", "Service:\n\"" + service_name +
                                      "\"\nresponded \"" + response->message +
                                      "\".");
        }
      });
}

void WavemapMapDisplay::requestWholeMapCallback() {
  ProfilerZoneScoped;
  // Resolve name of the service based on the map topic
  const std::string map_topic = topic_property_->getTopicStd();
  const std::optional<std::string> service_name =
      map_topic.empty() ? std::nullopt
                        : std::optional{map_topic + kRequestFullMapSuffix};
  if (!connectServiceClient<std_srvs::srv::Empty>(map_topic, service_name,
                                                  request_whole_map_client_)) {
    return;
  }

  // Call the service
  // NOTE: The callback overload is used even though the response is empty,
  //       since it also removes the request from the client's pending list.
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  request_whole_map_client_->async_send_request(
      request, [](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture) {});
}

void WavemapMapDisplay::loadMapFromDiskCallback() {
  ProfilerZoneScoped;
  // Open file selection dialog
  const auto filepath_qt = QFileDialog::getOpenFileName();

  // Check if the chosen filepath is not empty
  if (filepath_qt.isEmpty()) {
    load_map_from_disk_property_.resetAllValues();
    return;
  }

  // Load the map
  const std::filesystem::path filepath{filepath_qt.toStdString()};
  if (!loadMapFromDisk(filepath)) {
    load_map_from_disk_property_.resetAllValues();
    return;
  }

  // Update the button property to show the map's name (when not in focus)
  load_map_from_disk_property_.setAtRestValue(filepath.filename());

  // Update the visuals
  updateVisuals(true);
}

void WavemapMapDisplay::showAlertLater(const std::string& title,
                                       const std::string& description) {
  // NOTE: Passing `this` as the context cancels the alert if the display is
  //       destroyed before the event loop gets to it.
  QTimer::singleShot(0, this, [title, description]() {
    AlertDialog alert{title, description};
    alert.exec();
  });
}

std::optional<std::string>
WavemapMapDisplay::resolveWavemapServerNamespaceFromMapTopic(
    const std::string& map_topic, const std::string& child_topic) {
  ProfilerZoneScoped;
  const auto pos = map_topic.rfind('/');
  if (pos == std::string::npos) {
    return std::nullopt;
  }

  std::string wavemap_server_namespace = map_topic.substr(0, pos);
  if (child_topic.empty()) {
    return wavemap_server_namespace;
  } else {
    return wavemap_server_namespace + "/" + child_topic;
  }
}
}  // namespace wavemap::rviz_plugin

// Tell pluginlib about this class.
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(wavemap::rviz_plugin::WavemapMapDisplay,
                       rviz_common::Display)
