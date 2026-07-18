# SURF multi-robot mapping and communication simulation

Two differential-drive robots, each carrying a 16-layer 3D GPU LiDAR, maintain
robot-local Bonxai maps and exchange selected voxel-map changes over emulated
Wi-Fi HaLow and 5 GHz Wi-Fi links.

Target platform:

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic

## Install dependencies

```bash
sudo apt update
sudo apt install \
  ros-jazzy-ros-gz \
  ros-jazzy-rviz2 \
  ros-jazzy-tf2-ros \
  ros-jazzy-teleop-twist-keyboard \
  libzstd-dev
```

## Put the package in your workspace

```bash
cd ~/SURF_2026/surf_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to surf_multirobot_sim
source install/setup.bash
```

Replace the placeholder maintainer email in `package.xml` whenever convenient.

## Run

```bash
ros2 launch surf_multirobot_sim two_robots.launch.py
```

The launch file starts Gazebo, both robots, FAST-LIO, two local Bonxai servers,
the communication senders and receivers, and four directional link emulators.

The real-time path uses the HaLow profile with a freshness-first queue of depth
two. Periodic full refreshes and retained deletion tombstones use the reliable
5 GHz synchronization path.

## Verify

```bash
ros2 topic list | grep -E 'robot1|robot2'
ros2 topic hz /robot1/points
ros2 topic hz /robot2/points
ros2 topic echo /robot1/odom --once
ros2 topic echo /robot1/comm/pipeline_metrics --once
ros2 topic echo /robot1/comm/halow_metrics --once
ros2 topic bw /robot1/points
ros2 topic bw /robot1/comm/halow_tx
```

Gazebo-side check:

```bash
gz topic -l | grep -E 'robot1|robot2' | grep -E 'cmd_vel|odometry|points'
```

## Drive the robots

Robot 1:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/robot1/cmd_vel
```

Robot 2, in another terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/robot2/cmd_vel
```

## Communication architecture

```text
robot1 LiDAR -> robot1 local Bonxai
             -> voxel selector -> SVD1/zstd -> HaLow emulator ----+
             -> periodic refresh -> SVD1/zstd -> Wi-Fi emulator --+-> robot2 receiver
                                                              -> robot2 local Bonxai

robot2 uses the same paths in the opposite direction.
```

The sparse protocol is defined in `surf_multirobot_msgs`. Every packet carries
a source, map epoch, current and base versions, map frame, resolution, operating
mode, traffic class, and encoded payload. Decoded records explicitly encode:

- persistent occupied voxels;
- dynamic occupied voxels;
- observed free voxels;
- deletion/reset tombstones.

The sender filters invalid, out-of-range, self, vertically excluded, duplicate,
static-map-redundant, and unchanged voxels. A completed scan is processed by a
background worker while the ROS callback retains only the newest waiting scan.
Unseen dynamic cells expire after `dynamic_retention_scans` and become retained
free-space tombstones, so a later full refresh cannot resurrect stale geometry.

## Adaptive modes

The sender uses an EWMA of the emulator's advertised capacity, queue pressure,
hysteresis, separate up/down hold times, and a minimum dwell time. Thresholds
are in `config/communication.yaml`.

| Mode | Payload |
|---|---|
| `FULL` | Every valid voxel each scan |
| `VOXEL_DELTAS` | New, changed, cleared, deleted, or refresh-due voxels |
| `DYNAMIC_ONLY` | Fresh non-static geometry plus clears/deletes |
| `METADATA_ONLY` | Versioned heartbeat with no geometry |

Set `adaptive.manual_mode` to `0`, `1`, `2`, or `3` for controlled experiments;
use `-1` for adaptation.

## Link profiles and trace replay

Without arguments, the simulation uses a conservative 4 Mbps HaLow test
profile and an 80 Mbps synchronization profile. These are experiment defaults,
not claims about a particular radio.

Replay the included changing profiles:

```bash
ros2 launch surf_multirobot_sim two_robots.launch.py \
  halow_trace:=$PWD/src/surf_multirobot_sim/config/link_traces/halow_example.csv \
  wifi_trace:=$PWD/src/surf_multirobot_sim/config/link_traces/wifi_example.csv
```

For hardware, sample throughput with `iperf3` and RTT/loss with `ping` at a
fixed interval. Save:

```text
time_s,throughput_mbps,rtt_ms,loss_percent,jitter_ms
```

Convert it to the replay schema:

```bash
ros2 run surf_multirobot_sim prepare_link_trace.py measured_halow.csv halow.csv
```

Then pass `halow_trace:=/absolute/path/halow.csv`. Actual 802.11ah testing still
requires the selected HaLow radios, antennas, channel widths, regulatory domain,
and deployment environment; the simulator provides the repeatable replay side
of that workflow.

## View both maps in RViz

```bash
rviz2 --ros-args -p use_sim_time:=true
```

In RViz:

1. Set **Fixed Frame** to `world`.
2. Add a **TF** display.
3. Add **PointCloud2** displays.
4. Select `/robot1/bonxai/occupied_voxels` and
   `/robot2/bonxai/occupied_voxels` for the independently maintained maps.
5. Set PointCloud2 **Style** to `Points` and increase **Size (Pixels)** if needed.

## Important implementation details

- The robot SDF does not hard-code command, odometry, or LiDAR topic names.
- Gazebo scopes those topics using the spawned model name, so the same SDF can be spawned as `robot1` and `robot2`.
- The GPU LiDAR publishes a laser scan and a point-cloud topic ending in `/scan/points`.
- `bridge.yaml` maps those verbose Gazebo topics to `/robot1/points` and `/robot2/points`.
- Each Bonxai server consumes its own sensor directly and applies remote
  occupied/free/delete deltas through a typed ingress—not a reconstructed fake
  point cloud. Remote evidence is isolated by source and only unioned for fused
  outputs, so a remote delete cannot erase locally observed occupancy.
- Every sender epoch starts with a reliable full refresh. Later full refreshes
  atomically replace that source's layer, which reconciles packet loss and
  removes state absent from the snapshot.
- Coarser communication voxels are expanded across all covered local Bonxai
  cells when sender and map resolutions differ.
- Gazebo truth currently supplies each `map -> robotN/odom` correction. Replace
  `map_odom_localizer` with real global/cooperative localization on hardware.
- Bonxai's persistent map file is loaded by both local servers but is not saved
  automatically, preventing two processes from writing the same file.

## Performance tuning

If Gazebo runs slowly, edit `models/diffbot/model.sdf` in this order:

1. Change horizontal samples from `720` to `360`.
2. Change vertical samples from `16` to `8`.
3. Change update rate from `10` to `5`.
4. Change maximum range from `50.0` to `25.0`.

## Metrics

`PipelineMetrics` reports incoming scan rate, raw and valid point counts,
selected voxel and operation counts, exact raw-cloud and output CDR sizes,
encoded payload size, payload compression ratio, compression/total processing
latency, cumulative raw/wire bytes, and stale input replacements. Tiny SVD1
deltas are sent raw when a zstd frame would make them larger; larger deltas use
zstd. Ray clearing is uniformly capped by `maximum_clear_rays` to bound sender
latency, and the sampled subset rotates between scans for coverage. `LinkMetrics` reports
configured conditions, delivered throughput, observed delivery latency, queue
depth, packet loss, stale replacements, and transmitted bytes. Physical-radio
headers and retransmissions must be measured on hardware; the emulator adds a
configurable per-packet overhead estimate.
