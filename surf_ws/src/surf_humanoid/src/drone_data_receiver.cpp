#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"

#include "surf_multirobot_msgs/msg/compressed_voxel_delta.hpp"
#include "surf_multirobot_msgs/msg/delivery_metrics.hpp"
#include "surf_multirobot_msgs/msg/voxel_delta.hpp"
#include "surf_humanoid/voxel_codec.hpp"

namespace surf_humanoid
{

class DroneDataReceiver : public rclcpp::Node
{
public:
  DroneDataReceiver()
  : Node("drone_data_receiver")
  {
    robot_name_ = declare_parameter<std::string>("robot_name", "humanoid");
    realtime_topic_ = declare_parameter<std::string>(
      "realtime_topic", "/" + robot_name_ + "/comm/halow_rx");
    sync_topic_ = declare_parameter<std::string>(
      "sync_topic", "/" + robot_name_ + "/comm/wifi_rx");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/" + robot_name_ + "/comm/drone_voxel_delta");
    metrics_topic_ = declare_parameter<std::string>(
      "metrics_topic", "/" + robot_name_ + "/comm/delivery_metrics");
    maximum_uncompressed_bytes_ = static_cast<std::size_t>(std::max<int64_t>(
      1024, declare_parameter<int64_t>("maximum_uncompressed_bytes", 64 * 1024 * 1024)));

    publisher_ = create_publisher<surf_multirobot_msgs::msg::VoxelDelta>(
      output_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());
    metrics_publisher_ = create_publisher<surf_multirobot_msgs::msg::DeliveryMetrics>(
      metrics_topic_, rclcpp::QoS(10));
    realtime_subscription_ = create_subscription<
      surf_multirobot_msgs::msg::CompressedVoxelDelta>(
      realtime_topic_, rclcpp::QoS(rclcpp::KeepLast(2)).best_effort().durability_volatile(),
      std::bind(&DroneDataReceiver::receive, this, std::placeholders::_1));
    sync_subscription_ = create_subscription<surf_multirobot_msgs::msg::CompressedVoxelDelta>(
      sync_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
      std::bind(&DroneDataReceiver::receive, this, std::placeholders::_1));

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
    const auto receive_start = std::chrono::steady_clock::now();
    surf_multirobot_msgs::msg::DeliveryMetrics metrics;
    metrics.header.stamp = now();
    metrics.source_id = packet->source_id;
    metrics.map_epoch = packet->map_epoch;
    metrics.version = packet->version;
    metrics.traffic_class = packet->traffic_class;
    metrics.voxel_count = packet->voxel_count;
    rclcpp::Serialization<surf_multirobot_msgs::msg::CompressedVoxelDelta> serializer;
    rclcpp::SerializedMessage serialized;
    serializer.serialize_message(packet.get(), &serialized);
    metrics.wire_bytes = static_cast<uint32_t>(serialized.size());
    const rclcpp::Time sent(packet->header.stamp);
    metrics.transport_latency_ms = static_cast<float>((now() - sent).seconds() * 1000.0);
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
      metrics.accepted = false;
      metrics.rejection_reason = result.error;
      metrics.decode_latency_ms = static_cast<float>(std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - receive_start).count());
      metrics.end_to_end_latency_ms = metrics.transport_latency_ms + metrics.decode_latency_ms;
      metrics_publisher_->publish(metrics);
      RCLCPP_WARN(get_logger(), "Rejected voxel packet from %s: %s",
        packet->source_id.c_str(), result.error.c_str());
      return;
    }

    publisher_->publish(delta);
    metrics.accepted = true;
    metrics.decode_latency_ms = static_cast<float>(std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - receive_start).count());
    metrics.end_to_end_latency_ms = metrics.transport_latency_ms + metrics.decode_latency_ms;
    metrics_publisher_->publish(metrics);
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
  std::string metrics_topic_;
  std::size_t maximum_uncompressed_bytes_{64U * 1024U * 1024U};
  std::unordered_map<std::string, SourceState> sources_;

  rclcpp::Publisher<surf_multirobot_msgs::msg::VoxelDelta>::SharedPtr publisher_;
  rclcpp::Publisher<surf_multirobot_msgs::msg::DeliveryMetrics>::SharedPtr metrics_publisher_;
  rclcpp::Subscription<surf_multirobot_msgs::msg::CompressedVoxelDelta>::SharedPtr
    realtime_subscription_;
  rclcpp::Subscription<surf_multirobot_msgs::msg::CompressedVoxelDelta>::SharedPtr
    sync_subscription_;
};

}  // namespace surf_humanoid

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<surf_humanoid::DroneDataReceiver>());
  rclcpp::shutdown();
  return 0;
}
