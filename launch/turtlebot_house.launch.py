#!/usr/bin/env python3
""" Launch file for starting a turtlebot3 in the house.

Author: Charlie Street
Owner: Charlie Street
"""

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
from launch import LaunchDescription
from launch.actions import (
    SetEnvironmentVariable,
    IncludeLaunchDescription,
    DeclareLaunchArgument,
)
import os


def generate_launch_description():

    # Set sim time
    set_sim_time = SetParameter(name="use_sim_time", value=True)

    # Get package directories
    launch_file_dir = os.path.join(
        get_package_share_directory("turtlebot_house_sim"), "launch"
    )
    turtlebot_gazebo_dir = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"), "launch"
    )

    # Set all env vars and launch arguments
    set_turtle_model = SetEnvironmentVariable(name="TURTLEBOT3_MODEL", value="waffle")

    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")

    gui_arg = DeclareLaunchArgument("gui", default_value="true")
    sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    x_pose_arg = DeclareLaunchArgument("x_pose", default_value="-8.6894")
    y_pose_arg = DeclareLaunchArgument("y_pose", default_value="7.69215")

    # Start up the Gazebo simulator
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "house.launch.py")
        ),
        launch_arguments={"gui": gui}.items(),
    )

    # Start up the robot and corresponding state publisher
    start_robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot_gazebo_dir, "robot_state_publisher.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    start_turtlebot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot_gazebo_dir, "spawn_turtlebot3.launch.py")
        ),
        launch_arguments={"x_pose": x_pose, "y_pose": y_pose}.items(),
    )

    # Add all the commands
    ld = LaunchDescription()
    ld.add_action(set_sim_time)
    ld.add_action(set_turtle_model)
    ld.add_action(gui_arg)
    ld.add_action(sim_time_arg)
    ld.add_action(x_pose_arg)
    ld.add_action(y_pose_arg)
    ld.add_action(start_gazebo)
    ld.add_action(start_robot_state_publisher)
    ld.add_action(start_turtlebot_spawn)

    return ld
