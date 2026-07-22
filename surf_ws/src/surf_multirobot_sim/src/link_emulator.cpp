#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <fstream>
#include <memory>
#include <mutex>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"

#include "surf_multirobot_msgs/msg/compressed_voxel_delta.hpp"
#include "surf_multirobot_msgs/msg/link_metrics.hpp"

namespace surf_multirobot_sim
{

class LinkEmulator : public rclcpp::Node
{
public:
  LinkEmulator()
  : Node("link_emulator")
  {
    link_name_ = declare_parameter<std::string>("link_name", "halow");
    input_topic_ = declare_parameter<std::string>("input_topic", "/comm/tx");
    output_topic_ = declare_parameter<std::string>("output_topic", "/comm/rx");
    metrics_topic_ = declare_parameter<std::string>("metrics_topic", "/comm/link_metrics");
    profile_.bandwidth_mbps = declare_parameter<double>("bandwidth_mbps", 4.0);
    profile_.latency_ms = declare_parameter<double>("latency_ms", 25.0);
    profile_.jitter_ms = declare_parameter<double>("jitter_ms", 5.0);
    profile_.loss_percent = declare_parameter<double>("loss_percent", 0.5);
    queue_depth_limit_ = static_cast<int>(std::max<int64_t>(
      1, declare_parameter<int64_t>("queue_depth", 2)));
    freshness_first_ = declare_parameter<bool>("freshness_first", true);
    reliable_qos_ = declare_parameter<bool>("reliable_qos", false);
    packet_overhead_bytes_ = static_cast<int>(std::max<int64_t>(
      0, declare_parameter<int64_t>("packet_overhead_bytes", 96)));
    trace_path_ = declare_parameter<std::string>("trace_path", "");
    const int random_seed = static_cast<int>(declare_parameter<int64_t>("random_seed", 42));
    random_generator_.seed(static_cast<uint32_t>(random_seed));

    load_trace();
    start_time_ = std::chrono::steady_clock::now();
    next_transmit_time_ = start_time_;
    window_start_ = start_time_;

    rclcpp::QoS packet_qos(rclcpp::KeepLast(std::max(2, queue_depth_limit_)));
    packet_qos.durability_volatile();
    if (reliable_qos_) {
      packet_qos.reliable();
    } else {
      packet_qos.best_effort();
    }
    publisher_ = create_publisher<surf_multirobot_msgs::msg::CompressedVoxelDelta>(
      output_topic_, packet_qos);
    subscription_ = create_subscription<surf_multirobot_msgs::msg::CompressedVoxelDelta>(
      input_topic_, packet_qos,
      std::bind(&LinkEmulator::packet_callback, this, std::placeholders::_1));
    metrics_publisher_ = create_publisher<surf_multirobot_msgs::msg::LinkMetrics>(
      metrics_topic_, rclcpp::QoS(10));

    delivery_timer_ = create_wall_timer(
      std::chrono::milliseconds(1), std::bind(&LinkEmulator::deliver_ready, this));
    metrics_timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&LinkEmulator::publish_metrics, this));

    RCLCPP_INFO(get_logger(),
      "%s link: %s -> %s at %.3f Mbps, %.1f ms, %.2f%% loss%s",
      link_name_.c_str(), input_topic_.c_str(), output_topic_.c_str(),
      profile_.bandwidth_mbps, profile_.latency_ms, profile_.loss_percent,
      trace_.empty() ? "" : " (trace replay enabled)");
  }

private:
  using SteadyTime = std::chrono::steady_clock::time_point;

  struct Profile
  {
    double bandwidth_mbps{4.0};
    double latency_ms{25.0};
    double jitter_ms{5.0};
    double loss_percent{0.5};
  };

  struct TraceRow
  {
    double time_seconds{0.0};
    Profile profile;
  };

  struct PendingPacket
  {
    surf_multirobot_msgs::msg::CompressedVoxelDelta::SharedPtr message;
    SteadyTime enqueued;
    SteadyTime transmit_end;
    SteadyTime delivery;
    std::size_t accounted_bytes{0U};
  };

  void load_trace()
  {
    if (trace_path_.empty()) {
      return;
    }
    std::ifstream input(trace_path_);
    if (!input) {
      RCLCPP_WARN(get_logger(), "Could not open link trace: %s", trace_path_.c_str());
      return;
    }
    std::string line;
    std::size_t line_number = 0U;
    while (std::getline(input, line)) {
      ++line_number;
      if (line.empty() || line.front() == '#') {
        continue;
      }
      std::replace(line.begin(), line.end(), ',', ' ');
      std::istringstream row(line);
      TraceRow value;
      if (!(row >> value.time_seconds >> value.profile.bandwidth_mbps >>
        value.profile.latency_ms >> value.profile.jitter_ms >> value.profile.loss_percent))
      {
        if (line_number == 1U) {
          continue;
        }
        RCLCPP_WARN(get_logger(), "Skipping malformed trace row %zu", line_number);
        continue;
      }
      trace_.push_back(value);
    }
    std::sort(trace_.begin(), trace_.end(), [](const TraceRow & lhs, const TraceRow & rhs) {
      return lhs.time_seconds < rhs.time_seconds;
    });
  }

  void update_profile(const SteadyTime & now)
  {
    if (trace_.empty()) {
      return;
    }
    const double elapsed = std::chrono::duration<double>(now - start_time_).count();
    while (trace_index_ + 1U < trace_.size() &&
      trace_[trace_index_ + 1U].time_seconds <= elapsed)
    {
      ++trace_index_;
    }
    if (trace_[trace_index_].time_seconds <= elapsed) {
      profile_ = trace_[trace_index_].profile;
    }
  }

  std::size_t serialized_size(
    const surf_multirobot_msgs::msg::CompressedVoxelDelta & message) const
  {
    rclcpp::Serialization<surf_multirobot_msgs::msg::CompressedVoxelDelta> serializer;
    rclcpp::SerializedMessage serialized;
    serializer.serialize_message(&message, &serialized);
    return serialized.size() + static_cast<std::size_t>(packet_overhead_bytes_);
  }

  void packet_callback(surf_multirobot_msgs::msg::CompressedVoxelDelta::SharedPtr message)
  {
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    update_profile(now);
    ++received_packets_;

    const double loss_draw = uniform_(random_generator_) * 100.0;
    if (loss_draw < std::clamp(profile_.loss_percent, 0.0, 100.0)) {
      ++dropped_loss_;
      return;
    }

    if (static_cast<int>(queue_.size()) >= queue_depth_limit_) {
      if (!freshness_first_) {
        ++dropped_stale_;
        return;
      }
      queue_.pop_front();
      ++dropped_stale_;
      next_transmit_time_ = now;
      if (!queue_.empty()) {
        next_transmit_time_ = std::max(now, queue_.back().transmit_end);
      }
    }

    const std::size_t bytes = serialized_size(*message);
    const double bandwidth_bps = std::max(1.0, profile_.bandwidth_mbps * 1.0e6);
    const double transmit_seconds = static_cast<double>(bytes) * 8.0 / bandwidth_bps;
    const SteadyTime transmit_start = std::max(now, next_transmit_time_);
    next_transmit_time_ = transmit_start + std::chrono::duration_cast<SteadyTime::duration>(
      std::chrono::duration<double>(transmit_seconds));
    const double jitter = normal_(random_generator_) * std::max(0.0, profile_.jitter_ms);
    const double latency_ms = std::max(0.0, profile_.latency_ms + jitter);
    const SteadyTime delivery = next_transmit_time_ +
      std::chrono::duration_cast<SteadyTime::duration>(
      std::chrono::duration<double, std::milli>(latency_ms));
    queue_.push_back({std::move(message), now, next_transmit_time_, delivery, bytes});
  }

  void deliver_ready()
  {
    std::vector<PendingPacket> ready;
    const auto now = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      while (!queue_.empty() && queue_.front().delivery <= now) {
        ready.push_back(std::move(queue_.front()));
        queue_.pop_front();
      }
    }
    for (auto & packet : ready) {
      publisher_->publish(*packet.message);
      const double latency_ms = std::chrono::duration<double, std::milli>(
        now - packet.enqueued).count();
      std::lock_guard<std::mutex> lock(mutex_);
      ++delivered_packets_;
      transmitted_bytes_ += packet.accounted_bytes;
      window_bytes_ += packet.accounted_bytes;
      cumulative_delivery_latency_ms_ += latency_ms;
    }
  }

  void publish_metrics()
  {
    const auto now_steady = std::chrono::steady_clock::now();
    surf_multirobot_msgs::msg::LinkMetrics metrics;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      update_profile(now_steady);
      const double window_seconds = std::max(
        1.0e-6, std::chrono::duration<double>(now_steady - window_start_).count());
      metrics.header.stamp = now();
      metrics.link_name = link_name_;
      metrics.configured_bandwidth_mbps = static_cast<float>(profile_.bandwidth_mbps);
      metrics.configured_latency_ms = static_cast<float>(profile_.latency_ms);
      metrics.configured_jitter_ms = static_cast<float>(profile_.jitter_ms);
      metrics.configured_loss_percent = static_cast<float>(profile_.loss_percent);
      metrics.measured_throughput_mbps = static_cast<float>(
        static_cast<double>(window_bytes_) * 8.0 / window_seconds / 1.0e6);
      metrics.mean_delivery_latency_ms = delivered_packets_ > 0U ? static_cast<float>(
        cumulative_delivery_latency_ms_ / delivered_packets_) : 0.0F;
      metrics.queue_depth = static_cast<uint32_t>(queue_.size());
      metrics.received_packets = received_packets_;
      metrics.delivered_packets = delivered_packets_;
      metrics.dropped_loss = dropped_loss_;
      metrics.dropped_stale = dropped_stale_;
      metrics.transmitted_bytes = transmitted_bytes_;
      window_bytes_ = 0U;
      window_start_ = now_steady;
    }
    metrics_publisher_->publish(metrics);
  }

  std::string link_name_;
  std::string input_topic_;
  std::string output_topic_;
  std::string metrics_topic_;
  std::string trace_path_;
  Profile profile_;
  std::vector<TraceRow> trace_;
  std::size_t trace_index_{0U};
  int queue_depth_limit_{2};
  bool freshness_first_{true};
  bool reliable_qos_{false};
  int packet_overhead_bytes_{96};

  std::mutex mutex_;
  std::deque<PendingPacket> queue_;
  SteadyTime start_time_;
  SteadyTime next_transmit_time_;
  SteadyTime window_start_;
  std::mt19937 random_generator_;
  std::uniform_real_distribution<double> uniform_{0.0, 1.0};
  std::normal_distribution<double> normal_{0.0, 1.0};

  uint64_t received_packets_{0U};
  uint64_t delivered_packets_{0U};
  uint64_t dropped_loss_{0U};
  uint64_t dropped_stale_{0U};
  uint64_t transmitted_bytes_{0U};
  uint64_t window_bytes_{0U};
  double cumulative_delivery_latency_ms_{0.0};

  rclcpp::Subscription<surf_multirobot_msgs::msg::CompressedVoxelDelta>::SharedPtr subscription_;
  rclcpp::Publisher<surf_multirobot_msgs::msg::CompressedVoxelDelta>::SharedPtr publisher_;
  rclcpp::Publisher<surf_multirobot_msgs::msg::LinkMetrics>::SharedPtr metrics_publisher_;
  rclcpp::TimerBase::SharedPtr delivery_timer_;
  rclcpp::TimerBase::SharedPtr metrics_timer_;
};

}  // namespace surf_multirobot_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<surf_multirobot_sim::LinkEmulator>());
  rclcpp::shutdown();
  return 0;
}
