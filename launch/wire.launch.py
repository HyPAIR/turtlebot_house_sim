#!/usr/bin/env python3
""" Launch file for all wire-related behaviours.

Author: Charlie Street
Owner: Charlie Street
"""

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
import os


def generate_launch_description():

    # Set sim time
    set_sim_time = SetParameter(name="use_sim_time", value=True)

    # Get package root
    house_root = get_package_share_directory("turtlebot_house_sim")

    # Wire probability map and topological map
    prob_map_path = os.path.join(house_root, "maps/house_wire_prob_map.yaml")
    top_map_path = os.path.join(house_root, "maps/house_top_map.yaml")

    # All launch args
    top_map = LaunchConfiguration("top_map")
    wire_prob_map_yaml = LaunchConfiguration("wire_prob_map_yaml")
    wire_status_file = LaunchConfiguration("wire_status_file")
    wire_status_index = LaunchConfiguration("wire_status_index")

    top_map_arg = DeclareLaunchArgument("top_map", default_value=top_map_path)
    wire_prob_map_arg = DeclareLaunchArgument(
        "wire_prob_map_yaml", default_value=prob_map_path
    )
    wire_status_file_arg = DeclareLaunchArgument(
        "wire_status_file", default_value="not_set"
    )
    wire_status_index_arg = DeclareLaunchArgument(
        "wire_status_index", default_value="-1"
    )

    wire_manager_node = Node(
        package="turtlebot_house_sim",
        executable="wire_manager",
        name="wire_manager",
        parameters=[
            {
                "top_map": top_map,
                "wire_prob_map_yaml": wire_prob_map_yaml,
                "wire_status_file": wire_status_file,
                "wire_status_index": wire_status_index,
            }
        ],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(set_sim_time)
    ld.add_action(top_map_arg)
    ld.add_action(wire_prob_map_arg)
    ld.add_action(wire_status_file_arg)
    ld.add_action(wire_status_index_arg)
    ld.add_action(wire_manager_node)

    return ld
