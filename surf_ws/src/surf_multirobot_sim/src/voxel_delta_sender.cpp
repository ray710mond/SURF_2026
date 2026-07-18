#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "surf_multirobot_msgs/msg/compressed_voxel_delta.hpp"
#include "surf_multirobot_msgs/msg/link_metrics.hpp"
#include "surf_multirobot_msgs/msg/pipeline_metrics.hpp"
#include "surf_multirobot_msgs/msg/voxel_delta.hpp"
#include "surf_multirobot_sim/adaptive_mode.hpp"
#include "surf_multirobot_sim/voxel_codec.hpp"

namespace surf_multirobot_sim
{
namespace
{

struct Coord
{
  int32_t x{0};
  int32_t y{0};
  int32_t z{0};

  bool operator==(const Coord & other) const noexcept
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct CoordHash
{
  std::size_t operator()(const Coord & value) const noexcept
  {
    std::size_t seed = std::hash<int32_t>{}(value.x);
    seed ^= std::hash<int32_t>{}(value.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    seed ^= std::hash<int32_t>{}(value.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

struct CellState
{
  uint32_t consecutive_hits{0};
  uint32_t consecutive_misses{0};
  uint64_t last_seen_version{0};
  uint64_t last_sent_version{0};
  bool static_known{false};
  bool last_sent_static{false};
};

Coord quantize(double x, double y, double z, double resolution)
{
  return {
    static_cast<int32_t>(std::floor(x / resolution)),
    static_cast<int32_t>(std::floor(y / resolution)),
    static_cast<int32_t>(std::floor(z / resolution))};
}

void append_record(
  surf_multirobot_msgs::msg::VoxelDelta & delta,
  const Coord & coord, uint8_t state)
{
  delta.x.push_back(coord.x);
  delta.y.push_back(coord.y);
  delta.z.push_back(coord.z);
  delta.state.push_back(state);
}

double steady_seconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace

class VoxelDeltaSender : public rclcpp::Node
{
public:
  VoxelDeltaSender()
  : Node("voxel_delta_sender"),
    adaptive_(load_adaptive_config())
  {
    robot_name_ = declare_parameter<std::string>("robot_name", "robot1");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    input_topic_ = declare_parameter<std::string>("input_topic", "/" + robot_name_ + "/points");
    static_map_topic_ = declare_parameter<std::string>(
      "static_map_topic", "/" + robot_name_ + "/bonxai/static_occupied_voxels");
    realtime_topic_ = declare_parameter<std::string>(
      "realtime_topic", "/" + robot_name_ + "/comm/halow_tx");
    sync_topic_ = declare_parameter<std::string>(
      "sync_topic", "/" + robot_name_ + "/comm/wifi_tx");
    link_metrics_topic_ = declare_parameter<std::string>(
      "link_metrics_topic", "/" + robot_name_ + "/comm/halow_metrics");
    metrics_topic_ = declare_parameter<std::string>(
      "metrics_topic", "/" + robot_name_ + "/comm/pipeline_metrics");

    resolution_ = declare_parameter<double>("resolution", 0.20);
    min_range_ = declare_parameter<double>("filters.min_range", 0.75);
    max_range_ = declare_parameter<double>("filters.max_range", 50.0);
    min_z_ = declare_parameter<double>("filters.min_z", 0.25);
    max_z_ = declare_parameter<double>("filters.max_z", 20.0);
    self_radius_ = declare_parameter<double>("filters.self_radius", 0.70);
    static_min_hits_ = static_cast<int>(std::max<int64_t>(
      1, declare_parameter<int64_t>("filters.static_min_hits", 20)));
    clear_min_misses_ = static_cast<int>(std::max<int64_t>(
      1, declare_parameter<int64_t>("filters.clear_min_misses", 3)));
    delta_refresh_scans_ = static_cast<int>(std::max<int64_t>(
      1, declare_parameter<int64_t>("delta_refresh_scans", 50)));
    tombstone_retention_scans_ = static_cast<int>(std::max<int64_t>(
      1, declare_parameter<int64_t>("tombstone_retention_scans", 500)));
    dynamic_retention_scans_ = static_cast<int>(std::max<int64_t>(
      1, declare_parameter<int64_t>("dynamic_retention_scans", 20)));
    maximum_ray_voxels_ = static_cast<int>(std::max<int64_t>(
      1, declare_parameter<int64_t>("maximum_ray_voxels", 600)));
    maximum_clear_rays_ = static_cast<int>(std::max<int64_t>(
      1, declare_parameter<int64_t>("maximum_clear_rays", 256)));
    sync_interval_seconds_ = declare_parameter<double>("sync_interval_seconds", 10.0);
    compression_level_ = static_cast<int>(declare_parameter<int64_t>("compression_level", 1));
    adaptive_enabled_ = declare_parameter<bool>("adaptive.enabled", true);
    manual_mode_ = static_cast<int>(declare_parameter<int64_t>("adaptive.manual_mode", -1));

    std::random_device random_device;
    map_epoch_ = (static_cast<uint64_t>(random_device()) << 32U) ^ random_device();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto realtime_qos = rclcpp::QoS(rclcpp::KeepLast(2)).best_effort().durability_volatile();
    realtime_publisher_ = create_publisher<surf_multirobot_msgs::msg::CompressedVoxelDelta>(
      realtime_topic_, realtime_qos);
    sync_publisher_ = create_publisher<surf_multirobot_msgs::msg::CompressedVoxelDelta>(
      sync_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());
    metrics_publisher_ = create_publisher<surf_multirobot_msgs::msg::PipelineMetrics>(
      metrics_topic_, rclcpp::QoS(10));

    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS().keep_last(2),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr message) {
        const auto received = std::chrono::steady_clock::now();
        {
          std::lock_guard<std::mutex> lock(queue_mutex_);
          if (last_input_time_ != std::chrono::steady_clock::time_point{}) {
            const double interval = std::chrono::duration<double>(
              received - last_input_time_).count();
            if (interval > 0.0) {
              const double instantaneous_rate = 1.0 / interval;
              input_rate_hz_ = input_rate_hz_ == 0.0 ? instantaneous_rate :
                0.2 * instantaneous_rate + 0.8 * input_rate_hz_;
            }
          }
          last_input_time_ = received;
          if (latest_cloud_) {
            ++stale_input_drops_;
          }
          latest_cloud_ = std::move(message);
        }
        queue_condition_.notify_one();
      });
    static_map_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      static_map_topic_, rclcpp::QoS(1).reliable().durability_volatile(),
      std::bind(&VoxelDeltaSender::static_map_callback, this, std::placeholders::_1));
    link_metrics_subscription_ = create_subscription<surf_multirobot_msgs::msg::LinkMetrics>(
      link_metrics_topic_, rclcpp::QoS(10),
      std::bind(&VoxelDeltaSender::link_metrics_callback, this, std::placeholders::_1));

    last_sync_time_ = steady_seconds() - std::max(0.0, sync_interval_seconds_);
    worker_ = std::thread(&VoxelDeltaSender::worker_loop, this);
    RCLCPP_INFO(get_logger(),
      "%s communication sender: %s -> [%s, %s], resolution %.2fm",
      robot_name_.c_str(), input_topic_.c_str(), realtime_topic_.c_str(), sync_topic_.c_str(),
      resolution_);
  }

  ~VoxelDeltaSender() override
  {
    stop_.store(true);
    queue_condition_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  AdaptiveModeController::Config load_adaptive_config()
  {
    AdaptiveModeController::Config config;
    config.full_min_mbps = declare_parameter<double>("adaptive.full_min_mbps", 12.0);
    config.delta_min_mbps = declare_parameter<double>("adaptive.delta_min_mbps", 3.0);
    config.dynamic_min_mbps = declare_parameter<double>("adaptive.dynamic_min_mbps", 0.75);
    config.up_hysteresis_mbps = declare_parameter<double>("adaptive.up_hysteresis_mbps", 1.0);
    config.down_hold_seconds = declare_parameter<double>("adaptive.down_hold_seconds", 2.0);
    config.up_hold_seconds = declare_parameter<double>("adaptive.up_hold_seconds", 5.0);
    config.minimum_dwell_seconds = declare_parameter<double>("adaptive.minimum_dwell_seconds", 5.0);
    config.ewma_alpha = declare_parameter<double>("adaptive.ewma_alpha", 0.2);
    config.congested_queue_depth = static_cast<uint32_t>(std::max<int64_t>(
      1, declare_parameter<int64_t>("adaptive.congested_queue_depth", 2)));
    return config;
  }

  void worker_loop()
  {
    while (!stop_.load()) {
      sensor_msgs::msg::PointCloud2::SharedPtr cloud;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_condition_.wait(lock, [this]() {return stop_.load() || latest_cloud_ != nullptr;});
        if (stop_.load()) {
          return;
        }
        cloud = std::move(latest_cloud_);
      }
      process_cloud(*cloud);
    }
  }

  void static_map_callback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    {
      std::lock_guard<std::mutex> lock(static_map_mutex_);
      if (static_prior_initialized_) {
        return;
      }
    }
    std::unordered_set<Coord, CoordHash> replacement;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*message, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z)) {
          replacement.insert(quantize(*x, *y, *z, resolution_));
        }
      }
    } catch (const std::runtime_error & exception) {
      RCLCPP_WARN(get_logger(), "Ignoring invalid static voxel cloud: %s", exception.what());
      return;
    }
    std::lock_guard<std::mutex> lock(static_map_mutex_);
    if (!static_prior_initialized_) {
      static_map_ = std::move(replacement);
      static_prior_initialized_ = true;
      RCLCPP_INFO(get_logger(), "Captured %zu voxels in the static communication prior",
        static_map_.size());
    }
  }

  void link_metrics_callback(const surf_multirobot_msgs::msg::LinkMetrics::SharedPtr metrics)
  {
    std::lock_guard<std::mutex> lock(link_mutex_);
    available_bandwidth_mbps_ = metrics->configured_bandwidth_mbps;
    link_queue_depth_ = metrics->queue_depth;
  }

  uint8_t current_mode()
  {
    if (manual_mode_ >= 0 && manual_mode_ <= 3) {
      return static_cast<uint8_t>(manual_mode_);
    }
    if (!adaptive_enabled_) {
      return surf_multirobot_msgs::msg::VoxelDelta::MODE_VOXEL_DELTAS;
    }
    double bandwidth;
    uint32_t queue_depth;
    {
      std::lock_guard<std::mutex> lock(link_mutex_);
      bandwidth = available_bandwidth_mbps_;
      queue_depth = link_queue_depth_;
    }
    return adaptive_.update(steady_seconds(), bandwidth, queue_depth);
  }

  void trace_ray_for_clears(
    const Coord & origin, const Coord & endpoint,
    const std::unordered_set<Coord, CoordHash> & current,
    std::unordered_set<Coord, CoordHash> & traversed_known)
  {
    const int64_t dx = static_cast<int64_t>(endpoint.x) - origin.x;
    const int64_t dy = static_cast<int64_t>(endpoint.y) - origin.y;
    const int64_t dz = static_cast<int64_t>(endpoint.z) - origin.z;
    const int steps = static_cast<int>(std::min<int64_t>(
      maximum_ray_voxels_, std::max({std::llabs(dx), std::llabs(dy), std::llabs(dz)})));
    if (steps <= 1) {
      return;
    }
    Coord previous = origin;
    for (int step = 1; step < steps; ++step) {
      const double ratio = static_cast<double>(step) / static_cast<double>(steps);
      Coord coord{
        static_cast<int32_t>(std::llround(origin.x + ratio * dx)),
        static_cast<int32_t>(std::llround(origin.y + ratio * dy)),
        static_cast<int32_t>(std::llround(origin.z + ratio * dz))};
      if (coord == previous) {
        continue;
      }
      previous = coord;
      if (current.find(coord) == current.end() && cells_.find(coord) != cells_.end()) {
        traversed_known.insert(coord);
      }
    }
  }

  void process_cloud(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    const auto processing_start = std::chrono::steady_clock::now();
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
        map_frame_, cloud.header.frame_id, cloud.header.stamp,
        rclcpp::Duration::from_seconds(0.5));
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Communication filter cannot transform cloud: %s", exception.what());
      return;
    }

    tf2::Quaternion rotation;
    tf2::fromMsg(transform.transform.rotation, rotation);
    const tf2::Vector3 translation(
      transform.transform.translation.x,
      transform.transform.translation.y,
      transform.transform.translation.z);
    const Coord origin = quantize(translation.x(), translation.y(), translation.z(), resolution_);

    uint32_t raw_points = cloud.width * cloud.height;
    uint32_t valid_points = 0;
    std::unordered_set<Coord, CoordHash> current;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
          continue;
        }
        const tf2::Vector3 local(*x, *y, *z);
        const double range = local.length();
        if (range < min_range_ || range > max_range_) {
          continue;
        }
        const tf2::Vector3 mapped = tf2::quatRotate(rotation, local) + translation;
        if (mapped.z() < min_z_ || mapped.z() > max_z_ ||
          (mapped - translation).length2() < self_radius_ * self_radius_)
        {
          continue;
        }
        current.insert(quantize(mapped.x(), mapped.y(), mapped.z(), resolution_));
        ++valid_points;
      }
    } catch (const std::runtime_error & exception) {
      RCLCPP_ERROR(get_logger(), "Point cloud does not provide float x/y/z fields: %s",
        exception.what());
      return;
    }

    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> cloud_serializer;
    rclcpp::SerializedMessage serialized_cloud;
    cloud_serializer.serialize_message(&cloud, &serialized_cloud);
    const std::size_t raw_serialized_bytes = serialized_cloud.size();

    ++version_;
    const uint8_t mode = current_mode();
    surf_multirobot_msgs::msg::VoxelDelta realtime;
    initialize_delta(realtime, cloud, mode, false);
    realtime.sensor_origin.x = translation.x();
    realtime.sensor_origin.y = translation.y();
    realtime.sensor_origin.z = translation.z();

    std::unordered_set<Coord, CoordHash> static_snapshot;
    {
      std::lock_guard<std::mutex> lock(static_map_mutex_);
      static_snapshot = static_map_;
    }

    for (const auto & coord : current) {
      tombstones_.erase(coord);
      auto & cell = cells_[coord];
      const bool represented_by_prior = static_snapshot.find(coord) != static_snapshot.end();
      if (cell.last_seen_version + 1U == version_) {
        ++cell.consecutive_hits;
      } else {
        cell.consecutive_hits = 1U;
      }
      cell.consecutive_misses = 0U;
      cell.last_seen_version = version_;
      if (represented_by_prior || cell.consecutive_hits >= static_cast<uint32_t>(static_min_hits_))
      {
        cell.static_known = true;
      }

      const bool new_information = cell.last_sent_version == 0U ||
        cell.last_sent_static != cell.static_known;
      const bool refresh_due = version_ - cell.last_sent_version >=
        static_cast<uint64_t>(delta_refresh_scans_);
      bool send = mode == surf_multirobot_msgs::msg::VoxelDelta::MODE_FULL;
      if (mode == surf_multirobot_msgs::msg::VoxelDelta::MODE_VOXEL_DELTAS) {
        send = !represented_by_prior && (new_information || refresh_due);
      } else if (mode == surf_multirobot_msgs::msg::VoxelDelta::MODE_DYNAMIC_ONLY) {
        send = !represented_by_prior && !cell.static_known && (new_information || refresh_due);
      }
      if (send) {
        append_record(realtime, coord, cell.static_known ?
          surf_multirobot_msgs::msg::VoxelDelta::STATE_OCCUPIED_STATIC :
          surf_multirobot_msgs::msg::VoxelDelta::STATE_OCCUPIED_DYNAMIC);
        cell.last_sent_version = version_;
        cell.last_sent_static = cell.static_known;
      }
    }

    std::unordered_set<Coord, CoordHash> traversed_known;
    const std::size_t maximum_clear_rays = static_cast<std::size_t>(maximum_clear_rays_);
    const std::size_t ray_stride = std::max<std::size_t>(
      1U, (current.size() + maximum_clear_rays - 1U) / maximum_clear_rays);
    const std::size_t ray_offset = static_cast<std::size_t>(version_) % ray_stride;
    std::size_t ray_index = 0U;
    std::size_t sampled_rays = 0U;
    for (const auto & endpoint : current) {
      if ((ray_index++ % ray_stride) != ray_offset) {
        continue;
      }
      trace_ray_for_clears(origin, endpoint, current, traversed_known);
      if (++sampled_rays >= maximum_clear_rays) {
        break;
      }
    }
    for (const auto & coord : traversed_known) {
      auto found = cells_.find(coord);
      if (found == cells_.end()) {
        continue;
      }
      auto & cell = found->second;
      ++cell.consecutive_misses;
      if (cell.consecutive_misses < static_cast<uint32_t>(clear_min_misses_)) {
        continue;
      }
      const uint8_t cleared_state = cell.static_known ?
        surf_multirobot_msgs::msg::VoxelDelta::STATE_DELETE :
        surf_multirobot_msgs::msg::VoxelDelta::STATE_FREE;
      tombstones_[coord] = {version_, cleared_state};
      if (mode != surf_multirobot_msgs::msg::VoxelDelta::MODE_METADATA_ONLY) {
        append_record(realtime, coord, cleared_state);
      }
      cells_.erase(found);
    }

    // Dynamic cells that are no longer endpoints must not be resurrected by
    // periodic full refreshes merely because no sampled clearing ray hit them.
    for (auto it = cells_.begin(); it != cells_.end();) {
      const bool expired_dynamic = !it->second.static_known &&
        version_ - it->second.last_seen_version >
        static_cast<uint64_t>(dynamic_retention_scans_);
      if (!expired_dynamic) {
        ++it;
        continue;
      }
      tombstones_[it->first] = {
        version_, surf_multirobot_msgs::msg::VoxelDelta::STATE_FREE};
      if (mode != surf_multirobot_msgs::msg::VoxelDelta::MODE_METADATA_ONLY) {
        append_record(realtime, it->first,
          surf_multirobot_msgs::msg::VoxelDelta::STATE_FREE);
      }
      it = cells_.erase(it);
    }

    publish_delta(realtime,
      surf_multirobot_msgs::msg::CompressedVoxelDelta::TRAFFIC_REALTIME,
      *realtime_publisher_, raw_serialized_bytes, raw_points, valid_points, processing_start);

    const double now = steady_seconds();
    if (sync_interval_seconds_ > 0.0 && now - last_sync_time_ >= sync_interval_seconds_) {
      surf_multirobot_msgs::msg::VoxelDelta sync;
      initialize_delta(sync, cloud, mode, true);
      sync.sensor_origin = realtime.sensor_origin;
      sync.base_version = 0U;
      for (const auto & [coord, cell] : cells_) {
        append_record(sync, coord, cell.static_known ?
          surf_multirobot_msgs::msg::VoxelDelta::STATE_OCCUPIED_STATIC :
          surf_multirobot_msgs::msg::VoxelDelta::STATE_OCCUPIED_DYNAMIC);
      }
      for (auto it = tombstones_.begin(); it != tombstones_.end();) {
        if (version_ - it->second.version > static_cast<uint64_t>(tombstone_retention_scans_)) {
          it = tombstones_.erase(it);
          continue;
        }
        append_record(sync, it->first, it->second.state);
        ++it;
      }
      publish_delta(sync, surf_multirobot_msgs::msg::CompressedVoxelDelta::TRAFFIC_SYNC,
        *sync_publisher_, 0U, 0U, 0U, processing_start);
      last_sync_time_ = now;
    }
  }

  void initialize_delta(
    surf_multirobot_msgs::msg::VoxelDelta & delta,
    const sensor_msgs::msg::PointCloud2 & cloud, uint8_t mode, bool full_refresh)
  {
    delta.header = cloud.header;
    delta.header.frame_id = map_frame_;
    delta.source_id = robot_name_;
    delta.map_epoch = map_epoch_;
    delta.version = version_;
    delta.base_version = version_ > 0U ? version_ - 1U : 0U;
    delta.operating_mode = mode;
    delta.full_refresh = full_refresh;
    delta.resolution = static_cast<float>(resolution_);
  }

  void publish_delta(
    surf_multirobot_msgs::msg::VoxelDelta & delta, uint8_t traffic_class,
    rclcpp::Publisher<surf_multirobot_msgs::msg::CompressedVoxelDelta> & publisher,
    std::size_t raw_bytes, uint32_t raw_points, uint32_t valid_points,
    const std::chrono::steady_clock::time_point & processing_start)
  {
    surf_multirobot_msgs::msg::CompressedVoxelDelta wire;
    const auto compression_start = std::chrono::steady_clock::now();
    const CodecResult result = encode_delta(delta, wire, compression_level_);
    const float compression_latency_ms = static_cast<float>(
      std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - compression_start).count());
    if (!result.ok) {
      RCLCPP_ERROR(get_logger(), "Could not encode voxel delta: %s", result.error.c_str());
      return;
    }
    wire.traffic_class = traffic_class;

    rclcpp::Serialization<surf_multirobot_msgs::msg::CompressedVoxelDelta> serializer;
    rclcpp::SerializedMessage serialized;
    serializer.serialize_message(&wire, &serialized);
    const uint32_t wire_bytes = static_cast<uint32_t>(serialized.size());
    publisher.publish(wire);

    surf_multirobot_msgs::msg::PipelineMetrics metrics;
    metrics.header.stamp = now();
    metrics.header.frame_id = map_frame_;
    metrics.source_id = robot_name_;
    metrics.operating_mode = delta.operating_mode;
    metrics.traffic_class = traffic_class;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      metrics.input_rate_hz = static_cast<float>(input_rate_hz_);
      metrics.stale_input_drops = stale_input_drops_;
    }
    metrics.raw_points = raw_points;
    metrics.valid_points = valid_points;
    metrics.selected_voxels = static_cast<uint32_t>(delta.x.size());
    metrics.raw_serialized_bytes = static_cast<uint32_t>(raw_bytes);
    metrics.uncompressed_bytes = wire.uncompressed_bytes;
    metrics.payload_bytes = static_cast<uint32_t>(wire.payload.size());
    metrics.wire_bytes = wire_bytes;
    metrics.compression_ratio = !wire.payload.empty() ?
      static_cast<float>(wire.uncompressed_bytes) / wire.payload.size() : 0.0F;
    metrics.compression_latency_ms = compression_latency_ms;
    for (const uint8_t state : delta.state) {
      if (state == surf_multirobot_msgs::msg::VoxelDelta::STATE_FREE) {
        ++metrics.free_updates;
      } else if (state == surf_multirobot_msgs::msg::VoxelDelta::STATE_DELETE) {
        ++metrics.delete_updates;
      } else {
        ++metrics.occupied_updates;
      }
    }
    metrics.processing_latency_ms = static_cast<float>(
      std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - processing_start).count());
    cumulative_raw_bytes_ += raw_bytes;
    cumulative_wire_bytes_ += wire_bytes;
    metrics.cumulative_raw_bytes = cumulative_raw_bytes_;
    metrics.cumulative_wire_bytes = cumulative_wire_bytes_;
    metrics_publisher_->publish(metrics);
  }

  std::string robot_name_;
  std::string map_frame_;
  std::string input_topic_;
  std::string static_map_topic_;
  std::string realtime_topic_;
  std::string sync_topic_;
  std::string link_metrics_topic_;
  std::string metrics_topic_;
  double resolution_{0.2};
  double min_range_{0.75};
  double max_range_{50.0};
  double min_z_{0.25};
  double max_z_{20.0};
  double self_radius_{0.7};
  int static_min_hits_{20};
  int clear_min_misses_{3};
  int delta_refresh_scans_{50};
  int tombstone_retention_scans_{500};
  int dynamic_retention_scans_{20};
  int maximum_ray_voxels_{600};
  int maximum_clear_rays_{256};
  double sync_interval_seconds_{10.0};
  int compression_level_{1};
  bool adaptive_enabled_{true};
  int manual_mode_{-1};
  uint64_t map_epoch_{0U};
  uint64_t version_{0U};
  uint64_t cumulative_raw_bytes_{0U};
  uint64_t cumulative_wire_bytes_{0U};
  uint64_t stale_input_drops_{0U};
  double input_rate_hz_{0.0};
  std::chrono::steady_clock::time_point last_input_time_{};
  double last_sync_time_{0.0};

  AdaptiveModeController adaptive_;
  std::unordered_map<Coord, CellState, CoordHash> cells_;
  struct Tombstone
  {
    uint64_t version{0U};
    uint8_t state{surf_multirobot_msgs::msg::VoxelDelta::STATE_DELETE};
  };
  std::unordered_map<Coord, Tombstone, CoordHash> tombstones_;
  std::unordered_set<Coord, CoordHash> static_map_;
  bool static_prior_initialized_{false};
  std::mutex static_map_mutex_;

  double available_bandwidth_mbps_{4.0};
  uint32_t link_queue_depth_{0U};
  std::mutex link_mutex_;

  std::atomic<bool> stop_{false};
  std::thread worker_;
  std::mutex queue_mutex_;
  std::condition_variable queue_condition_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr static_map_subscription_;
  rclcpp::Subscription<surf_multirobot_msgs::msg::LinkMetrics>::SharedPtr
    link_metrics_subscription_;
  rclcpp::Publisher<surf_multirobot_msgs::msg::CompressedVoxelDelta>::SharedPtr
    realtime_publisher_;
  rclcpp::Publisher<surf_multirobot_msgs::msg::CompressedVoxelDelta>::SharedPtr
    sync_publisher_;
  rclcpp::Publisher<surf_multirobot_msgs::msg::PipelineMetrics>::SharedPtr metrics_publisher_;
};

}  // namespace surf_multirobot_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<surf_multirobot_sim::VoxelDeltaSender>());
  rclcpp::shutdown();
  return 0;
}
