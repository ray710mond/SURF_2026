#include <gtest/gtest.h>

#include <cstdint>

#include "surf_multirobot_msgs/msg/voxel_delta.hpp"
#include "surf_multirobot_sim/adaptive_mode.hpp"
#include "surf_multirobot_sim/voxel_codec.hpp"

namespace
{

TEST(VoxelCodec, RoundTripsEveryOperationAndSortsDeterministically)
{
  surf_multirobot_msgs::msg::VoxelDelta input;
  input.source_id = "robot1";
  input.map_epoch = 42U;
  input.version = 7U;
  input.base_version = 6U;
  input.resolution = 0.2F;
  input.x = {100, -3, 0, 100};
  input.y = {-200, 4, 0, -200};
  input.z = {5, -6, 0, 5};
  input.state = {
    surf_multirobot_msgs::msg::VoxelDelta::STATE_OCCUPIED_STATIC,
    surf_multirobot_msgs::msg::VoxelDelta::STATE_OCCUPIED_DYNAMIC,
    surf_multirobot_msgs::msg::VoxelDelta::STATE_FREE,
    surf_multirobot_msgs::msg::VoxelDelta::STATE_DELETE};

  surf_multirobot_msgs::msg::CompressedVoxelDelta wire;
  const auto encoded = surf_multirobot_sim::encode_delta(input, wire, 1);
  ASSERT_TRUE(encoded.ok) << encoded.error;
  EXPECT_EQ(wire.codec, "raw-svd1");
  EXPECT_LT(wire.payload.size(), 100U);

  surf_multirobot_msgs::msg::VoxelDelta output;
  const auto decoded = surf_multirobot_sim::decode_delta(wire, output);
  ASSERT_TRUE(decoded.ok) << decoded.error;
  EXPECT_EQ(output.source_id, input.source_id);
  EXPECT_EQ(output.map_epoch, input.map_epoch);
  EXPECT_EQ(output.version, input.version);
  ASSERT_EQ(output.x.size(), input.x.size());
  for (std::size_t index = 1; index < output.x.size(); ++index) {
    EXPECT_TRUE(output.x[index - 1] < output.x[index] ||
      (output.x[index - 1] == output.x[index] && output.y[index - 1] <= output.y[index]));
  }
}

TEST(VoxelCodec, RejectsMismatchedArraysAndCorruption)
{
  surf_multirobot_msgs::msg::VoxelDelta bad;
  bad.x.push_back(1);
  surf_multirobot_msgs::msg::CompressedVoxelDelta wire;
  EXPECT_FALSE(surf_multirobot_sim::encode_delta(bad, wire).ok);

  surf_multirobot_msgs::msg::VoxelDelta valid;
  valid.x = {1};
  valid.y = {2};
  valid.z = {3};
  valid.state = {surf_multirobot_msgs::msg::VoxelDelta::STATE_FREE};
  ASSERT_TRUE(surf_multirobot_sim::encode_delta(valid, wire).ok);
  wire.payload.at(0) ^= 0x5aU;
  surf_multirobot_msgs::msg::VoxelDelta output;
  EXPECT_FALSE(surf_multirobot_sim::decode_delta(wire, output).ok);
}

TEST(VoxelCodec, UsesZstdWhenItReducesPayloadSize)
{
  surf_multirobot_msgs::msg::VoxelDelta input;
  for (int32_t x = 0; x < 1000; ++x) {
    input.x.push_back(x);
    input.y.push_back(0);
    input.z.push_back(0);
    input.state.push_back(surf_multirobot_msgs::msg::VoxelDelta::STATE_OCCUPIED_STATIC);
  }

  surf_multirobot_msgs::msg::CompressedVoxelDelta wire;
  const auto encoded = surf_multirobot_sim::encode_delta(input, wire, 1);
  ASSERT_TRUE(encoded.ok) << encoded.error;
  EXPECT_EQ(wire.codec, "zstd-svd1");
  EXPECT_LT(wire.payload.size(), wire.uncompressed_bytes);

  surf_multirobot_msgs::msg::VoxelDelta output;
  const auto decoded = surf_multirobot_sim::decode_delta(wire, output);
  ASSERT_TRUE(decoded.ok) << decoded.error;
  EXPECT_EQ(output.x.size(), input.x.size());
}

TEST(AdaptiveMode, UsesHoldTimesAndHysteresis)
{
  surf_multirobot_sim::AdaptiveModeController::Config config;
  config.full_min_mbps = 10.0;
  config.delta_min_mbps = 3.0;
  config.dynamic_min_mbps = 1.0;
  config.up_hysteresis_mbps = 1.0;
  config.down_hold_seconds = 2.0;
  config.up_hold_seconds = 4.0;
  config.minimum_dwell_seconds = 0.0;
  config.ewma_alpha = 1.0;
  surf_multirobot_sim::AdaptiveModeController controller(config);

  EXPECT_EQ(controller.update(0.0, 5.0, 0U),
    surf_multirobot_msgs::msg::VoxelDelta::MODE_VOXEL_DELTAS);
  EXPECT_EQ(controller.update(1.0, 0.5, 0U),
    surf_multirobot_msgs::msg::VoxelDelta::MODE_VOXEL_DELTAS);
  EXPECT_EQ(controller.update(3.1, 0.5, 0U),
    surf_multirobot_msgs::msg::VoxelDelta::MODE_METADATA_ONLY);
  EXPECT_EQ(controller.update(4.0, 12.0, 0U),
    surf_multirobot_msgs::msg::VoxelDelta::MODE_METADATA_ONLY);
  EXPECT_EQ(controller.update(8.1, 12.0, 0U),
    surf_multirobot_msgs::msg::VoxelDelta::MODE_FULL);
}

TEST(AdaptiveMode, NeverUpgradesWhileTheLinkQueueIsCongested)
{
  surf_multirobot_sim::AdaptiveModeController::Config config;
  config.full_min_mbps = 10.0;
  config.delta_min_mbps = 3.0;
  config.dynamic_min_mbps = 1.0;
  config.up_hysteresis_mbps = 0.0;
  config.down_hold_seconds = 0.0;
  config.up_hold_seconds = 0.0;
  config.minimum_dwell_seconds = 0.0;
  config.ewma_alpha = 1.0;
  config.congested_queue_depth = 2U;
  surf_multirobot_sim::AdaptiveModeController controller(config);

  EXPECT_EQ(controller.update(0.0, 0.1, 2U),
    surf_multirobot_msgs::msg::VoxelDelta::MODE_VOXEL_DELTAS);
  EXPECT_EQ(controller.update(0.1, 0.1, 2U),
    surf_multirobot_msgs::msg::VoxelDelta::MODE_METADATA_ONLY);
  EXPECT_EQ(controller.update(0.2, 20.0, 2U),
    surf_multirobot_msgs::msg::VoxelDelta::MODE_METADATA_ONLY);
}

}  // namespace
