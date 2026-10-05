#ifndef WAVEMAP_RVIZ_PLUGIN_ROS2_WAVEMAP_MAP_DISPLAY_H_
#define WAVEMAP_RVIZ_PLUGIN_ROS2_WAVEMAP_MAP_DISPLAY_H_

#ifndef Q_MOC_RUN
#include <chrono>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>

#include <rclcpp/client.hpp>
#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/property.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <wavemap2_msgs/msg/map.hpp>

#include "wavemap2_rviz_plugin/common.h"
#include "wavemap2_rviz_plugin/utils/button_property.h"
#include "wavemap2_rviz_plugin/visuals/slice_visual.h"
#include "wavemap2_rviz_plugin/visuals/voxel_visual.h"
#endif

namespace wavemap::rviz_plugin {
struct SourceMode : public TypeSelector<SourceMode> {
  using TypeSelector<SourceMode>::TypeSelector;

  enum Id : TypeId { kFromTopic, kFromFile };

  static constexpr std::array names = {"Topic", "File"};
};

// The WavemapMapDisplay class implements the editable parameters and Display
// subclass machinery. The visuals themselves are represented by a separate
// class, WavemapMapVisual. The idiom for the visuals is that when the
// objects exist, they appear in the scene, and when they are deleted, they
// disappear.
class WavemapMapDisplay
    : public rviz_common::MessageFilterDisplay<wavemap2_msgs::msg::Map> {
  Q_OBJECT
 public:  // NOLINT
  // Constructor. pluginlib::ClassLoader creates instances by calling
  // the default constructor, so make sure you have one.
  WavemapMapDisplay();
  ~WavemapMapDisplay() override = default;

 protected:
  void onInitialize() override;

  // A helper to clear this display back to the initial state.
  void reset() override;

 private Q_SLOTS:  // NOLINT
  // These Qt slots get connected to signals indicating changes in the
  // user-editable properties
  void updateSourceModeCallback();
  void requestWavemapServerResetCallback();
  void requestWholeMapCallback();
  void loadMapFromDiskCallback();

 private:
  SourceMode source_mode_ = SourceMode::kFromTopic;

  bool hasMap();
  void clearMap();
  bool loadMapFromDisk(const std::filesystem::path& filepath);
  void updateVisuals(bool redraw_all = false);

  // Function to handle an incoming ROS message
  void processMessage(
      wavemap2_msgs::msg::Map::ConstSharedPtr map_msg) override;

  // Storage and message parsers for the map
  const std::shared_ptr<MapAndMutex> map_and_mutex_ =
      std::make_shared<MapAndMutex>();
  void updateMapFromRosMsg(const wavemap2_msgs::msg::Map& map_msg);

  // Submenus for each visual's properties
  rviz_common::properties::EnumProperty source_mode_property_{
      "Source", "", "Where to load the map from.", this,
      SLOT(updateSourceModeCallback())};
  ButtonProperty request_whole_map_property_{
      "Full map update",
      "Request",
      "Send a request to the wavemap_server to republish the whole map, "
      "instead of only increments.",
      this,
      SLOT(requestWholeMapCallback()),
      this};
  ButtonProperty load_map_from_disk_property_{
      "Loaded map",
      "Choose file",
      "Open a file dialog to choose and load a map from disk.",
      this,
      SLOT(loadMapFromDiskCallback()),
      this};
  rviz_common::properties::Property voxel_visual_properties_{
      "Render voxels", QVariant(),
      "Properties for the voxel-based visualization.", this};
  rviz_common::properties::Property slice_visual_properties_{
      "Render slice", QVariant(), "Properties for the slice visualization.",
      this};
  ButtonProperty request_wavemap_server_reset_property_{
      "Reset server",
      "Request",
      "Send a request to the wavemap_server to reset the map.",
      this,
      SLOT(requestWavemapServerResetCallback()),
      this,
      Qt::red};

  // Service clients to call wavemap's reset and full map republish services
  inline static const std::string kResetWavemapServerService = "reset_map";
  // NOTE: The server advertises the full map republish service under the map
  //       topic's name plus this suffix (see PublishMapOperation). The ROS1
  //       plugin called "<namespace>/republish_whole_map" instead, which no
  //       server advertises, so its "Full map update" button never connected.
  inline static const std::string kRequestFullMapSuffix = "_request_full";
  static std::optional<std::string> resolveWavemapServerNamespaceFromMapTopic(
      const std::string& map_topic, const std::string& child_topic = "");
  // How long to wait for a service to be discovered before reporting it
  // as unavailable
  static constexpr std::chrono::milliseconds kServiceWaitTime{500};
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr
      request_wavemap_server_reset_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr request_whole_map_client_;
  template <typename ServiceT>
  bool connectServiceClient(
      const std::string& map_topic,
      const std::optional<std::string>& service_name,
      typename rclcpp::Client<ServiceT>::SharedPtr& client);

  // Show an alert once control returns to Qt's event loop
  // NOTE: Alerts raised from ROS callbacks must not be shown directly. RViz2
  //       spins its executor on the GUI thread, so a modal dialog's nested
  //       event loop would call spin_some() again from within the callback,
  //       which rclcpp rejects.
  void showAlertLater(const std::string& title, const std::string& description);

  // Storage for the visuals
  // NOTE: Visuals are enabled when they are allocated, and automatically
  //       removed from the scene when destructed.
  std::unique_ptr<VoxelVisual> voxel_visual_;
  std::unique_ptr<SliceVisual> slice_visual_;
};
}  // namespace wavemap::rviz_plugin

#endif  // WAVEMAP_RVIZ_PLUGIN_ROS2_WAVEMAP_MAP_DISPLAY_H_
