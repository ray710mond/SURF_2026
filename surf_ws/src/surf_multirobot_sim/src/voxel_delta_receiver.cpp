#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"

#include "surf_multirobot_msgs/msg/compressed_voxel_delta.hpp"
#include "surf_multirobot_msgs/msg/voxel_delta.hpp"
#include "surf_multirobot_sim/voxel_codec.hpp"

namespace surf_multirobot_sim
{

class VoxelDeltaReceiver : public rclcpp::Node
{
public:
  VoxelDeltaReceiver()
  : Node("voxel_delta_receiver")
  {
    robot_name_ = declare_parameter<std::string>("robot_name", "robot2");
    realtime_topic_ = declare_parameter<std::string>(
      "realtime_topic", "/" + robot_name_ + "/comm/halow_rx");
    sync_topic_ = declare_parameter<std::string>(
      "sync_topic", "/" + robot_name_ + "/comm/wifi_rx");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/" + robot_name_ + "/comm/voxel_delta");
    maximum_uncompressed_bytes_ = static_cast<std::size_t>(std::max<int64_t>(
      1024, declare_parameter<int64_t>("maximum_uncompressed_bytes", 64 * 1024 * 1024)));

    publisher_ = create_publisher<surf_multirobot_msgs::msg::VoxelDelta>(
      output_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());
    realtime_subscription_ = create_subscription<
      surf_multirobot_msgs::msg::CompressedVoxelDelta>(
      realtime_topic_, rclcpp::QoS(rclcpp::KeepLast(2)).best_effort().durability_volatile(),
      std::bind(&VoxelDeltaReceiver::receive, this, std::placeholders::_1));
    sync_subscription_ = create_subscription<surf_multirobot_msgs::msg::CompressedVoxelDelta>(
      sync_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
      std::bind(&VoxelDeltaReceiver::receive, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "%s receiver: [%s, %s] -> %s",
      robot_name_.c_str(), realtime_topic_.c_str(), sync_topic_.c_str(), output_topic_.c_str());
  }

private:
  struct SourceState
  {
    uint64_t epoch{0U};
    uint64_t last_version{0U};
    uint64_t last_full_refresh_version{0U};
    bool awaiting_full_refresh{true};
    bool initialized{false};
  };

  void publish_epoch_reset(
    const surf_multirobot_msgs::msg::CompressedVoxelDelta & packet)
  {
    surf_multirobot_msgs::msg::VoxelDelta reset;
    reset.header = packet.header;
    reset.source_id = packet.source_id;
    reset.map_epoch = packet.map_epoch;
    reset.version = 0U;
    reset.base_version = 0U;
    reset.operating_mode = packet.operating_mode;
    reset.full_refresh = true;
    reset.resolution = packet.resolution;
    reset.sensor_origin = packet.sensor_origin;
    publisher_->publish(reset);
  }

  void receive(const surf_multirobot_msgs::msg::CompressedVoxelDelta::SharedPtr packet)
  {
    auto & source = sources_[packet->source_id];
    if (!source.initialized || source.epoch != packet->map_epoch) {
      if (source.initialized) {
        RCLCPP_WARN(get_logger(), "Map epoch changed for %s; awaiting refresh from new sender",
          packet->source_id.c_str());
      }
      publish_epoch_reset(*packet);
      source = SourceState{packet->map_epoch, 0U, 0U, true, true};
    }

    if (source.awaiting_full_refresh && !packet->full_refresh) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Waiting for a full refresh from %s", packet->source_id.c_str());
      return;
    }

    if (packet->full_refresh) {
      if (packet->version < source.last_full_refresh_version ||
        packet->version < source.last_version)
      {
        return;
      }
    } else {
      if (packet->version <= source.last_version) {
        return;
      }
      if (source.last_version > 0U && packet->base_version > source.last_version) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Version gap for %s: have %lu, received base %lu/version %lu",
          packet->source_id.c_str(), source.last_version, packet->base_version, packet->version);
      }
    }

    surf_multirobot_msgs::msg::VoxelDelta delta;
    const CodecResult result = decode_delta(*packet, delta, maximum_uncompressed_bytes_);
    if (!result.ok) {
      RCLCPP_WARN(get_logger(), "Rejected voxel packet from %s: %s",
        packet->source_id.c_str(), result.error.c_str());
      return;
    }

    publisher_->publish(delta);
    source.last_version = std::max(source.last_version, packet->version);
    if (packet->full_refresh) {
      source.last_full_refresh_version = packet->version;
      source.awaiting_full_refresh = false;
    }
  }

  std::string robot_name_;
  std::string realtime_topic_;
  std::string sync_topic_;
  std::string output_topic_;
  std::size_t maximum_uncompressed_bytes_{64U * 1024U * 1024U};
  std::unordered_map<std::string, SourceState> sources_;

  rclcpp::Publisher<surf_multirobot_msgs::msg::VoxelDelta>::SharedPtr publisher_;
  rclcpp::Subscription<surf_multirobot_msgs::msg::CompressedVoxelDelta>::SharedPtr
    realtime_subscription_;
  rclcpp::Subscription<surf_multirobot_msgs::msg::CompressedVoxelDelta>::SharedPtr
    sync_subscription_;
};

}  // namespace surf_multirobot_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<surf_multirobot_sim::VoxelDeltaReceiver>());
  rclcpp::shutdown();
  return 0;
}
