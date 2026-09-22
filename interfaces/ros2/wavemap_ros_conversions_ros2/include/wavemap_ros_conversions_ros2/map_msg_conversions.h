#ifndef WAVEMAP_ROS_CONVERSIONS_ROS2_MAP_MSG_CONVERSIONS_H_
#define WAVEMAP_ROS_CONVERSIONS_ROS2_MAP_MSG_CONVERSIONS_H_

#include <algorithm>
#include <memory>
#include <stack>
#include <string>
#include <unordered_set>
#include <utility>

#include <rclcpp/time.hpp>
#include <wavemap/core/map/hashed_blocks.h>
#include <wavemap/core/map/hashed_chunked_wavelet_octree.h>
#include <wavemap/core/map/hashed_wavelet_octree.h>
#include <wavemap/core/map/volumetric_octree.h>
#include <wavemap/core/map/wavelet_octree.h>
#include <wavemap/core/utils/thread_pool.h>
#include <wavemap_msgs_ros2/msg/map.hpp>

// Mechanical port of interfaces/ros1/wavemap_ros_conversions, same algorithms
// throughout. Only the ROS-specific surface changed: ros::Time -> rclcpp::Time
// and wavemap_msgs::* -> wavemap_msgs_ros2::msg::*.
namespace wavemap::convert {
bool mapToRosMsg(const MapBase& map, const std::string& frame_id,
                 const rclcpp::Time& stamp, wavemap_msgs_ros2::msg::Map& msg);
bool rosMsgToMap(const wavemap_msgs_ros2::msg::Map& msg, MapBase::Ptr& map);

void mapToRosMsg(const HashedBlocks& map,
                 wavemap_msgs_ros2::msg::HashedBlocks& msg);
void rosMsgToMap(const wavemap_msgs_ros2::msg::HashedBlocks& msg,
                 HashedBlocks::Ptr& map);

void mapToRosMsg(const WaveletOctree& map,
                 wavemap_msgs_ros2::msg::WaveletOctree& msg);
void rosMsgToMap(const wavemap_msgs_ros2::msg::WaveletOctree& msg,
                 WaveletOctree::Ptr& map);

void mapToRosMsg(const HashedWaveletOctree& map,
                 wavemap_msgs_ros2::msg::HashedWaveletOctree& msg,
                 std::optional<std::unordered_set<Index3D, Index3DHash>>
                     include_blocks = std::nullopt,
                 std::shared_ptr<ThreadPool> thread_pool = nullptr);
void blockToRosMsg(const HashedWaveletOctree::BlockIndex& block_index,
                   const HashedWaveletOctree::Block& block,
                   FloatingPoint min_log_odds, FloatingPoint max_log_odds,
                   wavemap_msgs_ros2::msg::HashedWaveletOctreeBlock& msg);
void rosMsgToMap(const wavemap_msgs_ros2::msg::HashedWaveletOctree& msg,
                 HashedWaveletOctree::Ptr& map);

void mapToRosMsg(const HashedChunkedWaveletOctree& map,
                 wavemap_msgs_ros2::msg::HashedWaveletOctree& msg,
                 std::optional<std::unordered_set<Index3D, Index3DHash>>
                     include_blocks = std::nullopt,
                 std::shared_ptr<ThreadPool> thread_pool = nullptr);
void blockToRosMsg(const HashedChunkedWaveletOctree::BlockIndex& block_index,
                   const HashedChunkedWaveletOctree::Block& block,
                   FloatingPoint min_log_odds, FloatingPoint max_log_odds,
                   wavemap_msgs_ros2::msg::HashedWaveletOctreeBlock& msg);
}  // namespace wavemap::convert

#endif  // WAVEMAP_ROS_CONVERSIONS_ROS2_MAP_MSG_CONVERSIONS_H_
