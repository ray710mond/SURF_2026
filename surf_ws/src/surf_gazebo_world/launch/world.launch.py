"""Launch the standalone SURF Gazebo world."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_share = Path(get_package_share_directory("surf_gazebo_world"))
    world_path = package_share / "worlds" / "surf_world.sdf"
    gui_config_path = package_share / "config" / "no_side_bar.config"
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))
    resource_path = str(package_share / "models")
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH")
    if existing_resource_path:
        resource_path += os.pathsep + existing_resource_path
    sdf_path = str(package_share / "models")
    existing_sdf_path = os.environ.get("SDF_PATH")
    if existing_sdf_path:
        sdf_path += os.pathsep + existing_sdf_path

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(ros_gz_share / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": f"-r --gui-config {gui_config_path} {world_path}"
        }.items(),
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
            SetEnvironmentVariable("SDF_PATH", sdf_path),
            gazebo,
        ]
    )
