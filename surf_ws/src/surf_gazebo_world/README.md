# surf_gazebo_world

Standalone Gazebo Harmonic world assets for the SURF ROS 2 workspace. The
current world reconstructs the Caltech area around Throop Memorial Garden from
overhead imagery and geolocated street-level photo spheres.

## Build

```bash
cd surf_ws
colcon build --packages-select surf_gazebo_world
source install/setup.bash
```

## Launch

```bash
ros2 launch surf_gazebo_world world.launch.py
```

To run the complete multi-robot mapping and communication stack in this world:

```bash
ros2 launch surf_multirobot_sim two_robots.launch.py
```

The local world frame uses X east, Y north, and Z up. Its geodetic origin is
34.136878 N, 118.124792 W at 245 m elevation. Photo-sphere coordinates were
converted to this local frame to place the main paths, pond, building facades,
and fixtures. Less-visible perimeter building dimensions remain approximate.

The world composes reusable models from `models/`, including the detailed
historic laboratory facade, landscaped garden, two tree types, shrubs, rocks,
benches, and illuminated lamp posts. Edit `worlds/surf_world.sdf` for site
layout and the individual model files for object-level refinements.
