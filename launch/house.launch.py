#!/usr/bin/env python3
""" This launch file has been modified from the file that appears in aws-robomaker-bookstore-world.

Author: Mostly AWS.
Owner: Charlie Street
"""

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import SetParameter
import launch
import os


def generate_launch_description():
    world_file_name = "house_interior.world"
    world = os.path.join(
        get_package_share_directory("turtlebot_house_sim"),
        "worlds",
        world_file_name,
    )
    gazebo_ros = get_package_share_directory("gazebo_ros")
    gazebo_client = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, "launch", "gzclient.launch.py")
        ),
        condition=launch.conditions.IfCondition(
            launch.substitutions.LaunchConfiguration("gui")
        ),
    )
    gazebo_server = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, "launch", "gzserver.launch.py")
        )
    )

    ld = launch.LaunchDescription(
        [
            SetParameter(name="use_sim_time", value=True),
            launch.actions.DeclareLaunchArgument(
                "world", default_value=[world, ""], description="SDF world file"
            ),
            launch.actions.DeclareLaunchArgument(name="gui", default_value="false"),
            gazebo_server,
            gazebo_client,
        ]
    )
    return ld


if __name__ == "__main__":
    generate_launch_description()
