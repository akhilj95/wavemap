#ifndef WAVEMAP_RVIZ_PLUGIN_ROS2_VISUALS_VOXEL_VISUAL_H_
#define WAVEMAP_RVIZ_PLUGIN_ROS2_VISUALS_VOXEL_VISUAL_H_

#ifndef Q_MOC_RUN
#include <memory>
#include <unordered_map>
#include <vector>

#include <Ogre.h>
#include <OgreQuaternion.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/view_manager.hpp>
#include <wavemap/core/config/type_selector.h>
#include <wavemap/core/indexing/index_hashes.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap/core/utils/time/time.h>

#include "wavemap_rviz_plugin_ros2/common.h"
#include "wavemap_rviz_plugin_ros2/utils/color_conversions.h"
#include "wavemap_rviz_plugin_ros2/utils/listeners.h"
#include "wavemap_rviz_plugin_ros2/visuals/cell_layer.h"
#include "wavemap_rviz_plugin_ros2/visuals/cell_selector.h"
#endif

namespace wavemap::rviz_plugin {
struct VoxelColorMode : public TypeSelector<VoxelColorMode> {
  using TypeSelector<VoxelColorMode>::TypeSelector;

  enum Id : TypeId { kHeight, kProbability, kFlat };

  static constexpr std::array names = {"Height", "Probability", "Flat"};
};

// Each instance of VoxelVisual represents the map's leaves
// as voxels whose sizes match their height in the tree.
class VoxelVisual : public QObject {
  Q_OBJECT
 public:  // NOLINT
  // Constructor. Creates the visual elements and puts them into the
  // scene, in an unconfigured state.
  VoxelVisual(Ogre::SceneManager* scene_manager,
              rviz_common::ViewManager* view_manager,
              Ogre::SceneNode* parent_node,
              rviz_common::properties::Property* submenu_root_property,
              std::shared_ptr<MapAndMutex> map_and_mutex);

  // Destructor. Removes the visual elements from the scene.
  ~VoxelVisual() override;

  void updateMap(bool redraw_all = false);

  void clear() { block_voxel_layers_map_.clear(); }

  // Set the pose of the coordinate frame the message refers to
  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);

 private Q_SLOTS:  // NOLINT
  // These Qt slots get connected to signals indicating changes in the
  // user-editable properties
  void visibilityUpdateCallback();
  void terminationHeightUpdateCallback() { force_lod_update_ = true; }
  void opacityUpdateCallback();
  void colorModeUpdateCallback();
  void flatColorUpdateCallback();

 private:
  VoxelColorMode voxel_color_mode_ = VoxelColorMode::kHeight;
  Ogre::ColourValue voxel_flat_color_ = Ogre::ColourValue::Blue;

  // Shared pointer to the map, owned by WavemapMapDisplay
  const std::shared_ptr<MapAndMutex> map_and_mutex_;

  // The SceneManager, kept here only so the destructor can ask it to
  // destroy the `frame_node_`.
  Ogre::SceneManager* scene_manager_;

  // A SceneNode whose pose is set to match the coordinate frame of
  // the WavemapOctree message header.
  Ogre::SceneNode* frame_node_;

  // User-editable property variables, contained in the visual's submenu
  // Visibility
  rviz_common::properties::BoolProperty visibility_property_;
  // Cell selection
  CellSelector cell_selector_;
  rviz_common::properties::IntProperty termination_height_property_;
  // Colors
  rviz_common::properties::FloatProperty opacity_property_;
  rviz_common::properties::EnumProperty color_mode_property_;
  rviz_common::properties::ColorProperty flat_color_property_;
  // Frame-rate stats
  rviz_common::properties::Property frame_rate_properties_;
  rviz_common::properties::IntProperty num_queued_blocks_indicator_;
  rviz_common::properties::IntProperty max_ms_per_frame_property_;

  // The objects implementing the voxel visuals
  using VoxelLayers = std::vector<std::unique_ptr<CellLayer>>;
  std::unordered_map<Index3D, VoxelLayers, Index3DHash> block_voxel_layers_map_;

  // Material handling
  Ogre::MaterialPtr voxel_material_;
  void setAlpha(FloatingPoint alpha);

  // Level of Detail control
  std::unique_ptr<ViewportPrerenderListener> prerender_listener_;
  void prerenderCallback(Ogre::Camera* active_camera);
  float lod_update_distance_threshold_ = 0.1f;
  Ogre::Vector3 camera_position_at_last_lod_update_{};
  bool force_lod_update_ = true;
  void updateLOD(const Ogre::Camera& active_camera);
  static IndexElement computeRecommendedBlockLodHeight(
      const Ogre::Camera& active_camera, const OctreeIndex& block_index,
      FloatingPoint min_cell_width, IndexElement min_height,
      IndexElement max_height);
  std::optional<IndexElement> getCurrentBlockLodHeight(
      IndexElement map_tree_height, const Index3D& block_idx);

  // Drawing related methods
  using VoxelsPerLevel = std::vector<std::vector<Cell>>;
  void appendLeafCenterAndColor(int tree_height, FloatingPoint min_cell_width,
                                const OctreeIndex& cell_index,
                                FloatingPoint cell_log_odds,
                                VoxelsPerLevel& voxels_per_level);
  void drawMultiResolutionVoxels(IndexElement tree_height,
                                 FloatingPoint min_cell_width,
                                 const Index3D& block_index,
                                 FloatingPoint alpha,
                                 VoxelsPerLevel& voxels_per_level,
                                 VoxelLayers& voxel_layer_visuals);

  // Block update queue
  // NOTE: Instead of performing all the block updates at once whenever the map
  //       is updated or the LOD levels change (due to camera motion), we add
  //       the changed blocks to the block_update_queue_. Blocks are then popped
  //       from the queue and updated until max_ms_per_frame_property_ is
  //       reached. Any blocks that have not yet been processed will then be
  //       updated in the next prerender cycle. This avoids excessive frame rate
  //       drops when large changes occur.
  Timestamp last_update_time_{};
  std::unordered_map<Index3D, IndexElement, Index3DHash> block_update_queue_;
  void processBlockUpdateQueue(const Point3D& camera_position);
};
}  // namespace wavemap::rviz_plugin

#endif  // WAVEMAP_RVIZ_PLUGIN_ROS2_VISUALS_VOXEL_VISUAL_H_
