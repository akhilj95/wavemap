#ifndef WAVEMAP2_ROS_CONVERSIONS_MAP_MSG_LAYOUT_H_
#define WAVEMAP2_ROS_CONVERSIONS_MAP_MSG_LAYOUT_H_

#include <cstddef>
#include <string_view>
#include <type_traits>
#include <utility>

#include <wavemap2_msgs/msg/hashed_wavelet_octree_block.hpp>
#include <wavemap2_msgs/msg/wavelet_octree.hpp>

namespace wavemap::convert {
// Whether an octree msg stores its nodes in the flat layout, with one primitive
// array per node field, rather than in the nested WaveletOctreeNode[] layout
// NOTE: The layout is chosen when wavemap2_msgs is built (see
//       WAVEMAP2_MSGS_LAYOUT in its CMakeLists.txt). Detecting it from the
//       generated types means packages that use them need no build flag.
template <typename OctreeMsgT, typename = void>
struct HasFlatNodeLayout : std::false_type {};
template <typename OctreeMsgT>
struct HasFlatNodeLayout<
    OctreeMsgT,
    std::void_t<decltype(std::declval<OctreeMsgT&>().node_detail_coefficients)>>
    : std::true_type {};

// The layout this build's wavemap2_msgs uses
inline constexpr bool kMapMsgsUseFlatNodeLayout =
    HasFlatNodeLayout<wavemap2_msgs::msg::HashedWaveletOctreeBlock>::value;
static_assert(
    HasFlatNodeLayout<wavemap2_msgs::msg::WaveletOctree>::value ==
        kMapMsgsUseFlatNodeLayout,
    "All octree msgs in wavemap2_msgs should use the same node layout.");
inline constexpr std::string_view kMapMsgNodeLayoutName =
    kMapMsgsUseFlatNodeLayout ? "flat" : "nested";

// Number of detail coefficients each node takes up in the octree msgs
// NOTE: Octree nodes only have 7 detail coefficients. Both layouts reserve 8
//       per node anyway to match ROS1's float32[8] detail_coefficients, and
//       always leave the last one at 0.
inline constexpr size_t kNumDetailCoefficientsPerNodeMsg = 8;
}  // namespace wavemap::convert

#endif  // WAVEMAP2_ROS_CONVERSIONS_MAP_MSG_LAYOUT_H_
