# SURF multi-robot mapping and communication simulation

The `humanoid` uses a differential-drive simulation model and the second role
uses a fully holonomic six-DOF inspection robot. The existing `drone` ROS
namespace is retained for compatibility. Only its LiDAR data is filtered, voxelized, compressed,
and transmitted. The humanoid decompresses that stream and fuses it with its
own direct LiDAR input in one Bonxai map.

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

The launch file starts Gazebo, both robots, FAST-LIO for both roles, a local
Bonxai server for each role, the drone sender, the humanoid receiver, and two
one-way link emulators (HaLow realtime plus Wi-Fi synchronization).

Gazebo loads the Caltech / Throop Memorial Garden environment from the separate
`surf_gazebo_world` package. The ground model is defined in
`models/diffbot/model.sdf`; the six-DOF model is defined in
`models/six_dof_robot/model.sdf`. The latter starts two metres above the ground
and accepts body-frame X, Y, Z, roll, pitch, and yaw velocity commands.

The real-time path uses the HaLow profile with a freshness-first queue of depth
two. Periodic full refreshes and retained deletion tombstones use the reliable
5 GHz synchronization path.

Link bandwidth varies with the 3D distance between the robots. The configured
`bandwidth_mbps` is the rate at `reference_distance_m` (5 m in this launch), and
the emulator applies `bandwidth = reference_bandwidth * (reference_distance /
distance)^2`. `minimum_distance_m` prevents a singularity when robots overlap,
and `minimum_bandwidth_mbps` keeps transmission times finite at long range.
`maximum_bandwidth_mbps` caps the result at the radio's nominal peak rate, so
moving closer than the reference distance cannot produce unlimited bandwidth.
When a link trace is enabled, each trace bandwidth is treated as the reference
bandwidth before distance scaling.

## Verify

```bash
ros2 topic list | grep -E 'humanoid|drone'
ros2 topic hz /humanoid/points
ros2 topic hz /drone/points
ros2 topic echo /humanoid/odom --once
ros2 topic echo /drone/comm/pipeline_metrics --once
ros2 topic echo /drone/comm/halow_metrics --once
ros2 topic bw /humanoid/points
ros2 topic bw /drone/comm/halow_tx
ros2 topic echo /humanoid/comm/drone_voxel_delta --once
```

Gazebo-side check:

```bash
gz topic -l | grep -E 'humanoid|drone' | grep -E 'cmd_vel|odometry|points'
```

## Drive the robots

Humanoid:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/humanoid/cmd_vel
```

Six-DOF robot (still under the `/drone` namespace), in another terminal:

```bash
ros2 run surf_multirobot_sim six_dof_teleop.py
```

The controls are `W/S` for X, `A/D` for Y, `R/F` for Z (up/down), `U/J` for
roll, `I/K` for pitch, and `O/L` for yaw. Space publishes a stop command.

You can also command an exact six-axis velocity directly:

```bash
ros2 topic pub --once /drone/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 1.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

## Communication architecture

```text
drone LiDAR -> filters/voxel selector -> SVD1/zstd
                                     +-> HaLow emulator --+
                                     +-> Wi-Fi emulator ---+
                                                         v
humanoid LiDAR -------------------------------> humanoid Bonxai map
humanoid receiver -> decoded drone deltas ---> humanoid Bonxai map
```

The sparse protocol is defined in `surf_multirobot_msgs`. Every packet carries
a source, map epoch, current and base versions, map frame, resolution, operating
mode, traffic class, and encoded payload. Decoded records explicitly encode:

- persistent occupied voxels;
- dynamic occupied voxels;
- observed free voxels;
- deletion/reset tombstones.

The drone sender filters invalid, out-of-range, self, humanoid-model, vertically
excluded, duplicate, static-map-redundant, and unchanged voxels. A completed
scan is processed by a background worker while the ROS callback retains only
the newest waiting scan.
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
4. Select `/humanoid/bonxai/occupied_voxels` for the humanoid's fused local and
   drone-assisted map, and `/drone/bonxai/occupied_voxels` for the drone's
   complete local map. Raw scans remain separately available as `/humanoid/points`
   and `/drone/points`.
5. Set PointCloud2 **Style** to `Points` and increase **Size (Pixels)** if needed.

## Important implementation details

- The robot SDF does not hard-code command, odometry, or LiDAR topic names.
- Gazebo scopes those topics using the spawned model name, so the same SDF can be spawned as `humanoid` and `drone`.
- The GPU LiDAR publishes a laser scan and a point-cloud topic ending in `/scan/points`.
- `bridge.yaml` maps those verbose Gazebo topics to `/humanoid/points` and `/drone/points`.
- The humanoid Bonxai server consumes `/humanoid/points` directly and applies
  decoded drone occupied/free/delete deltas through its typed ingress—not a
  reconstructed fake point cloud. Remote evidence is isolated by source and
  unioned for fused outputs, so a drone delete cannot erase humanoid evidence.
- The drone Bonxai server consumes every `/drone/points` scan locally and does
  not ingest remote deltas. Its local map is independent of the communication
  sender; only selected delta records are compressed and transmitted.
- The sender uses timestamp-matched `/humanoid/odom` state and the configured
  humanoid model envelope to suppress voxels on the humanoid. The mask applies
  only to transmitted real-time and sync deltas; `/drone/points` still reaches
  the drone Bonxai map unchanged.
- Every sender epoch starts with a reliable full refresh. Later full refreshes
  atomically replace that source's layer, which reconciles packet loss and
  removes state absent from the snapshot.
- Coarser communication voxels are expanded across all covered local Bonxai
  cells when sender and map resolutions differ.
- Gazebo truth currently supplies each `map -> <role>/odom` correction. Replace
  `map_odom_localizer` with real global/cooperative localization on hardware.
- FAST-LIO supplies each `<role>/odom -> <role>/base_link` transform. Gazebo
  truth is paired with that same LIO odometry to calculate the simulation-only
  `map -> <role>/odom` correction.
- Bonxai's persistent map file belongs to the humanoid server. Drone map loading
  and saving are disabled so the two processes cannot overwrite the same file.

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
