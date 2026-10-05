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
#include <wavemap2_msgs/msg/map.hpp>

// Mechanical port of interfaces/ros1/wavemap_ros_conversions, same algorithms
// throughout. Only the ROS-specific surface changed: ros::Time -> rclcpp::Time
// and wavemap_msgs::* -> wavemap2_msgs::msg::*.
namespace wavemap::convert {
bool mapToRosMsg(const MapBase& map, const std::string& frame_id,
                 const rclcpp::Time& stamp, wavemap2_msgs::msg::Map& msg);
bool rosMsgToMap(const wavemap2_msgs::msg::Map& msg, MapBase::Ptr& map);

void mapToRosMsg(const HashedBlocks& map,
                 wavemap2_msgs::msg::HashedBlocks& msg);
void rosMsgToMap(const wavemap2_msgs::msg::HashedBlocks& msg,
                 HashedBlocks::Ptr& map);

void mapToRosMsg(const WaveletOctree& map,
                 wavemap2_msgs::msg::WaveletOctree& msg);
void rosMsgToMap(const wavemap2_msgs::msg::WaveletOctree& msg,
                 WaveletOctree::Ptr& map);

void mapToRosMsg(const HashedWaveletOctree& map,
                 wavemap2_msgs::msg::HashedWaveletOctree& msg,
                 std::optional<std::unordered_set<Index3D, Index3DHash>>
                     include_blocks = std::nullopt,
                 std::shared_ptr<ThreadPool> thread_pool = nullptr);
void blockToRosMsg(const HashedWaveletOctree::BlockIndex& block_index,
                   const HashedWaveletOctree::Block& block,
                   FloatingPoint min_log_odds, FloatingPoint max_log_odds,
                   wavemap2_msgs::msg::HashedWaveletOctreeBlock& msg);
void rosMsgToMap(const wavemap2_msgs::msg::HashedWaveletOctree& msg,
                 HashedWaveletOctree::Ptr& map);

void mapToRosMsg(const HashedChunkedWaveletOctree& map,
                 wavemap2_msgs::msg::HashedWaveletOctree& msg,
                 std::optional<std::unordered_set<Index3D, Index3DHash>>
                     include_blocks = std::nullopt,
                 std::shared_ptr<ThreadPool> thread_pool = nullptr);
void blockToRosMsg(const HashedChunkedWaveletOctree::BlockIndex& block_index,
                   const HashedChunkedWaveletOctree::Block& block,
                   FloatingPoint min_log_odds, FloatingPoint max_log_odds,
                   wavemap2_msgs::msg::HashedWaveletOctreeBlock& msg);
}  // namespace wavemap::convert

#endif  // WAVEMAP_ROS_CONVERSIONS_ROS2_MAP_MSG_CONVERSIONS_H_
