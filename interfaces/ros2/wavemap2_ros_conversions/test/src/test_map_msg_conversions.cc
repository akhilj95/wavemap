#include <bitset>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <wavemap/core/common.h>
#include <wavemap/core/map/hashed_chunked_wavelet_octree.h>
#include <wavemap/core/map/hashed_wavelet_octree.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap/core/map/wavelet_octree.h>
#include <wavemap/test/config_generator.h>
#include <wavemap/test/fixture_base.h>
#include <wavemap/test/geometry_generator.h>
#include <wavemap2_msgs/msg/map.hpp>

#include "wavemap2_ros_conversions/map_msg_conversions.h"
#include "wavemap2_ros_conversions/map_msg_layout.h"

namespace wavemap {
template <typename MapType>
class MapMsgConversionsTest : public FixtureBase,
                              public GeometryGenerator,
                              public ConfigGenerator {
 protected:
  void SetUp() override {
    // NOTE: RCL_ROS_TIME rather than rclcpp::Time's default RCL_SYSTEM_TIME,
    //       as produced by a real node's clock. rclcpp throws when comparing
    //       times of different clock types, see time_conversions.h.
    stamp = rclcpp::Clock{RCL_ROS_TIME}.now();
  }

  const std::string frame_id = "odom";
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  static constexpr FloatingPoint kAcceptableReconstructionError = 5e-2f;
};

using MapTypes =
    ::testing::Types<HashedBlocks, WaveletOctree, HashedWaveletOctree,
                     HashedChunkedWaveletOctree>;
TYPED_TEST_SUITE(MapMsgConversionsTest, MapTypes, );

TYPED_TEST(MapMsgConversionsTest, MetadataPreservation) {
  const auto config =
      ConfigGenerator::getRandomConfig<typename TypeParam::Config>();

  // Create the original map and make sure it matches the config
  typename TypeParam::ConstPtr map = std::make_shared<TypeParam>(config);
  ASSERT_EQ(map->getMinCellWidth(), config.min_cell_width);
  ASSERT_EQ(map->getMinLogOdds(), config.min_log_odds);
  ASSERT_EQ(map->getMaxLogOdds(), config.max_log_odds);
  if constexpr (!std::is_same_v<TypeParam, HashedBlocks>) {
    ASSERT_EQ(map->getTreeHeight(), config.tree_height);
  }

  // Convert to base pointer
  MapBase::ConstPtr map_base = map;
  ASSERT_EQ(map_base->getMinCellWidth(), config.min_cell_width);
  ASSERT_EQ(map_base->getMinLogOdds(), config.min_log_odds);
  ASSERT_EQ(map_base->getMaxLogOdds(), config.max_log_odds);

  // Serialize and deserialize
  wavemap2_msgs::msg::Map map_msg;
  ASSERT_TRUE(convert::mapToRosMsg(*map_base, TestFixture::frame_id,
                                   TestFixture::stamp, map_msg));
  MapBase::Ptr map_base_round_trip;
  ASSERT_TRUE(convert::rosMsgToMap(map_msg, map_base_round_trip));
  ASSERT_TRUE(map_base_round_trip);

  // Check the header
  EXPECT_EQ(map_msg.header.frame_id, TestFixture::frame_id);
  EXPECT_EQ(rclcpp::Time(map_msg.header.stamp, RCL_ROS_TIME),
            TestFixture::stamp);

  // TODO(victorr): Add option to deserialize into hashed chunked wavelet
  //                octrees, instead of implicitly converting them to regular
  //                hashed wavelet octrees.
  if constexpr (std::is_same_v<TypeParam, HashedChunkedWaveletOctree>) {
    HashedWaveletOctree::ConstPtr map_round_trip =
        std::dynamic_pointer_cast<HashedWaveletOctree>(map_base_round_trip);
    ASSERT_TRUE(map_round_trip);

    // Check that the metadata still matches the original config
    EXPECT_EQ(map_round_trip->getMinCellWidth(), config.min_cell_width);
    EXPECT_EQ(map_round_trip->getMinLogOdds(), config.min_log_odds);
    EXPECT_EQ(map_round_trip->getMaxLogOdds(), config.max_log_odds);
    EXPECT_EQ(map_round_trip->getTreeHeight(), config.tree_height);
  } else {
    typename TypeParam::ConstPtr map_round_trip =
        std::dynamic_pointer_cast<TypeParam>(map_base_round_trip);
    ASSERT_TRUE(map_round_trip);

    // Check that the metadata still matches the original config
    EXPECT_EQ(map_round_trip->getMinCellWidth(), config.min_cell_width);
    EXPECT_EQ(map_round_trip->getMinLogOdds(), config.min_log_odds);
    EXPECT_EQ(map_round_trip->getMaxLogOdds(), config.max_log_odds);
    if constexpr (!std::is_same_v<TypeParam, HashedBlocks>) {
      EXPECT_EQ(map_round_trip->getTreeHeight(), config.tree_height);
    }
  }
}

TYPED_TEST(MapMsgConversionsTest, InsertionAndLeafVisitor) {
  constexpr int kNumRepetitions = 3;
  for (int i = 0; i < kNumRepetitions; ++i) {
    // Create a random map
    const auto config =
        ConfigGenerator::getRandomConfig<typename TypeParam::Config>();
    TypeParam map_original(config);
    const std::vector<Index3D> random_indices =
        GeometryGenerator::getRandomIndexVector<3>(
            1000u, 2000u, Index3D::Constant(-5000), Index3D::Constant(5000));
    for (const Index3D& index : random_indices) {
      const FloatingPoint update = TestFixture::getRandomUpdate();
      map_original.addToCellValue(index, update);
    }
    map_original.prune();

    // Serialize and deserialize
    wavemap2_msgs::msg::Map map_msg;
    ASSERT_TRUE(convert::mapToRosMsg(map_original, TestFixture::frame_id,
                                     TestFixture::stamp, map_msg));
    MapBase::Ptr map_base_round_trip;
    ASSERT_TRUE(convert::rosMsgToMap(map_msg, map_base_round_trip));
    ASSERT_TRUE(map_base_round_trip);

    // Check that both maps contain the same leaves
    map_base_round_trip->forEachLeaf(
        [&map_original](const OctreeIndex& node_index,
                        FloatingPoint round_trip_value) {
          if constexpr (std::is_same_v<TypeParam, HashedBlocks>) {
            EXPECT_EQ(node_index.height, 0);
            EXPECT_NEAR(round_trip_value,
                        map_original.getValueOrDefault(node_index.position),
                        TestFixture::kAcceptableReconstructionError);
          } else {
            EXPECT_NEAR(round_trip_value, map_original.getCellValue(node_index),
                        TestFixture::kAcceptableReconstructionError);
          }
        });

    // TODO(victorr): Remove this special case once deserializing directly
    //                into HashedChunkedWaveletOctrees is supported
    if (std::is_same_v<TypeParam, HashedChunkedWaveletOctree>) {
      HashedWaveletOctree::ConstPtr map_round_trip =
          std::dynamic_pointer_cast<HashedWaveletOctree>(map_base_round_trip);
      ASSERT_TRUE(map_round_trip);

      map_original.forEachLeaf([&map_round_trip](const OctreeIndex& node_index,
                                                 FloatingPoint original_value) {
        EXPECT_NEAR(original_value, map_round_trip->getCellValue(node_index),
                    TestFixture::kAcceptableReconstructionError);
      });
    } else {
      typename TypeParam::ConstPtr map_round_trip =
          std::dynamic_pointer_cast<TypeParam>(map_base_round_trip);
      ASSERT_TRUE(map_round_trip);

      map_original.forEachLeaf([&map_round_trip](const OctreeIndex& node_index,
                                                 FloatingPoint original_value) {
        if constexpr (std::is_same_v<TypeParam, HashedBlocks>) {
          EXPECT_EQ(node_index.height, 0);
          EXPECT_NEAR(original_value,
                      map_round_trip->getValueOrDefault(node_index.position),
                      TestFixture::kAcceptableReconstructionError);
        } else {
          EXPECT_NEAR(original_value, map_round_trip->getCellValue(node_index),
                      TestFixture::kAcceptableReconstructionError);
        }
      });
    }
  }
}

// Layout independent access to the nodes of WaveletOctree and
// HashedWaveletOctreeBlock msgs, see map_msg_layout.h
template <typename OctreeMsgT>
size_t getNumNodes(const OctreeMsgT& msg) {
  if constexpr (convert::HasFlatNodeLayout<OctreeMsgT>::value) {
    return msg.node_allocated_children_bitsets.size();
  } else {
    return msg.nodes.size();
  }
}

template <typename OctreeMsgT>
uint8_t getNodeAllocatedChildrenBitset(const OctreeMsgT& msg, size_t node_idx) {
  if constexpr (convert::HasFlatNodeLayout<OctreeMsgT>::value) {
    return msg.node_allocated_children_bitsets[node_idx];
  } else {
    return msg.nodes[node_idx].allocated_children_bitset;
  }
}

template <typename OctreeMsgT>
float getNodeDetailCoefficient(const OctreeMsgT& msg, size_t node_idx,
                               size_t coefficient_idx) {
  if constexpr (convert::HasFlatNodeLayout<OctreeMsgT>::value) {
    return msg.node_detail_coefficients
        [node_idx * convert::kNumDetailCoefficientsPerNodeMsg +
         coefficient_idx];
  } else {
    return msg.nodes[node_idx].detail_coefficients[coefficient_idx];
  }
}

// Calls visitor on each WaveletOctree or HashedWaveletOctreeBlock msg in msg
template <typename VisitorT>
void forEachOctreeMsg(wavemap2_msgs::msg::Map& msg, VisitorT visitor) {
  for (auto& octree_msg : msg.wavelet_octree) {
    visitor(octree_msg);
  }
  for (auto& hashed_octree_msg : msg.hashed_wavelet_octree) {
    for (auto& block_msg : hashed_octree_msg.blocks) {
      visitor(block_msg);
    }
  }
}

template <typename MapType>
class OctreeMsgNodesTest : public MapMsgConversionsTest<MapType> {
 protected:
  std::shared_ptr<MapType> getRandomMap() {
    const auto config =
        ConfigGenerator::getRandomConfig<typename MapType::Config>();
    auto map = std::make_shared<MapType>(config);
    const std::vector<Index3D> random_indices =
        GeometryGenerator::getRandomIndexVector<3>(
            1000u, 2000u, Index3D::Constant(-5000), Index3D::Constant(5000));
    for (const Index3D& index : random_indices) {
      map->addToCellValue(index, this->getRandomUpdate());
    }
    map->prune();
    return map;
  }

  wavemap2_msgs::msg::Map getRandomMapMsg() {
    wavemap2_msgs::msg::Map map_msg;
    EXPECT_TRUE(convert::mapToRosMsg(*getRandomMap(), this->frame_id,
                                     this->stamp, map_msg));
    return map_msg;
  }
};

using OctreeMapTypes = ::testing::Types<WaveletOctree, HashedWaveletOctree,
                                        HashedChunkedWaveletOctree>;
TYPED_TEST_SUITE(OctreeMsgNodesTest, OctreeMapTypes, );

TYPED_TEST(OctreeMsgNodesTest, NodesFormValidOctrees) {
  auto map_msg = TestFixture::getRandomMapMsg();
  size_t num_octree_msgs = 0;
  forEachOctreeMsg(map_msg, [&num_octree_msgs](const auto& octree_msg) {
    ++num_octree_msgs;
    const size_t num_nodes = getNumNodes(octree_msg);
    // The root node is always serialized
    ASSERT_GE(num_nodes, 1u);
    if constexpr (convert::HasFlatNodeLayout<
                      std::decay_t<decltype(octree_msg)>>::value) {
      EXPECT_EQ(octree_msg.node_detail_coefficients.size(),
                convert::kNumDetailCoefficientsPerNodeMsg * num_nodes);
    }
    size_t num_announced_children = 0;
    for (size_t node_idx = 0; node_idx < num_nodes; ++node_idx) {
      num_announced_children +=
          std::bitset<8>(getNodeAllocatedChildrenBitset(octree_msg, node_idx))
              .count();
      // Nodes only have 7 detail coefficients, the slot for an 8th must stay 0
      EXPECT_EQ(getNodeDetailCoefficient(
                    octree_msg, node_idx,
                    convert::kNumDetailCoefficientsPerNodeMsg - 1),
                0.f);
    }
    // Each node except the root is announced by its parent
    EXPECT_EQ(num_announced_children + 1, num_nodes);
  });
  EXPECT_GT(num_octree_msgs, 0u);
}

TYPED_TEST(OctreeMsgNodesTest, MalformedNodesAreRejected) {
  enum class Corruption { kMissingNode, kExtraNode, kMissingCoefficient };
  for (const auto corruption :
       {Corruption::kMissingNode, Corruption::kExtraNode,
        Corruption::kMissingCoefficient}) {
    // Only the flat layout can have a coefficient count that doesn't match
    // its node count
    if (corruption == Corruption::kMissingCoefficient &&
        !convert::kMapMsgsUseFlatNodeLayout) {
      continue;
    }
    SCOPED_TRACE("Corruption " + std::to_string(static_cast<int>(corruption)));

    // Corrupt the first octree in the msg
    auto map_msg = TestFixture::getRandomMapMsg();
    bool corrupted = false;
    forEachOctreeMsg(map_msg, [corruption, &corrupted](auto& octree_msg) {
      if (std::exchange(corrupted, true)) {
        return;
      }
      if constexpr (convert::HasFlatNodeLayout<
                        std::decay_t<decltype(octree_msg)>>::value) {
        auto& coefficients = octree_msg.node_detail_coefficients;
        auto& bitsets = octree_msg.node_allocated_children_bitsets;
        switch (corruption) {
          case Corruption::kMissingNode:
            coefficients.resize(coefficients.size() -
                                convert::kNumDetailCoefficientsPerNodeMsg);
            bitsets.pop_back();
            break;
          case Corruption::kExtraNode:
            coefficients.resize(coefficients.size() +
                                convert::kNumDetailCoefficientsPerNodeMsg);
            bitsets.emplace_back(0);
            break;
          case Corruption::kMissingCoefficient:
            coefficients.pop_back();
            break;
        }
      } else {
        switch (corruption) {
          case Corruption::kMissingNode:
            octree_msg.nodes.pop_back();
            break;
          case Corruption::kExtraNode:
            octree_msg.nodes.emplace_back();
            break;
          case Corruption::kMissingCoefficient:
            break;
        }
      }
    });
    ASSERT_TRUE(corrupted);

    // Check that deserialization reports the error and drops the corrupted data
    MapBase::Ptr map_base_round_trip;
    EXPECT_FALSE(convert::rosMsgToMap(map_msg, map_base_round_trip));
    ASSERT_TRUE(map_base_round_trip);
    if constexpr (std::is_same_v<TypeParam, WaveletOctree>) {
      EXPECT_TRUE(map_base_round_trip->empty());
    } else {
      // For hashed maps, only the corrupted block should be dropped
      const auto map_round_trip =
          std::dynamic_pointer_cast<HashedWaveletOctree>(map_base_round_trip);
      ASSERT_TRUE(map_round_trip);
      const auto& block_msgs = map_msg.hashed_wavelet_octree.front().blocks;
      ASSERT_GE(block_msgs.size(), 2u);
      for (size_t block_idx = 0; block_idx < block_msgs.size(); ++block_idx) {
        const auto& offset = block_msgs[block_idx].root_node_offset;
        const Index3D block_index{offset.x, offset.y, offset.z};
        EXPECT_EQ(map_round_trip->hasBlock(block_index), block_idx != 0);
      }
    }
  }
}
}  // namespace wavemap
