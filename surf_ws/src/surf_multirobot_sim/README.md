# SURF multi-robot Gazebo starter

Two differential-drive robots, each carrying a 16-layer 3D GPU LiDAR, in a simple outdoor Gazebo Harmonic environment.

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
  ros-jazzy-teleop-twist-keyboard
```

## Put the package in your workspace

```bash
mkdir -p ~/SURF_2026/src
cp -r surf_multirobot_sim ~/SURF_2026/src/
cd ~/SURF_2026
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select surf_multirobot_sim
source install/setup.bash
```

Replace the placeholder maintainer email in `package.xml` whenever convenient.

## Run

```bash
ros2 launch surf_multirobot_sim two_robots.launch.py
```

The launch file starts Gazebo, spawns `robot1` and `robot2`, starts all ROS–Gazebo bridges, and publishes the known initial transforms that place both odometry frames under `world`.

## Verify

```bash
ros2 topic list | grep -E 'robot1|robot2'
ros2 topic hz /robot1/points
ros2 topic hz /robot2/points
ros2 topic echo /robot1/odom --once
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

## View both clouds in RViz

```bash
rviz2 --ros-args -p use_sim_time:=true
```

In RViz:

1. Set **Fixed Frame** to `world`.
2. Add a **TF** display.
3. Add two **PointCloud2** displays.
4. Select `/robot1/points` and `/robot2/points`.
5. Set PointCloud2 **Style** to `Points` and increase **Size (Pixels)** if needed.

## Important implementation details

- The robot SDF does not hard-code command, odometry, or LiDAR topic names.
- Gazebo scopes those topics using the spawned model name, so the same SDF can be spawned as `robot1` and `robot2`.
- The GPU LiDAR publishes a laser scan and a point-cloud topic ending in `/scan/points`.
- `bridge.yaml` maps those verbose Gazebo topics to `/robot1/points` and `/robot2/points`.
- The `world -> robotN/odom` transforms encode the known spawn poses. This is useful for the first shared-map prototype but is not a replacement for multi-robot localization.

## Performance tuning

If Gazebo runs slowly, edit `models/diffbot/model.sdf` in this order:

1. Change horizontal samples from `720` to `360`.
2. Change vertical samples from `16` to `8`.
3. Change update rate from `10` to `5`.
4. Change maximum range from `50.0` to `25.0`.

## Next step

Transform `/robot1/points` and `/robot2/points` into `world`, combine them, voxel-filter the result, and publish `/shared/points`. Then feed that cloud into an occupancy representation such as OctoMap.
