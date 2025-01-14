#!/usr/bin/env python3
""" Launch file for the policy executor.

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

    root_dir = get_package_share_directory("turtlebot_house_sim")
    top_map_path = os.path.join(root_dir, "maps/house_top_map.yaml")
    prob_map_path = os.path.join(root_dir, "maps/house_wire_prob_map.yaml")

    # All launch args
    db_connection_string = LaunchConfiguration("db_connection_string")
    db_name = LaunchConfiguration("db_name")
    db_collection = LaunchConfiguration("db_collection")
    top_map = LaunchConfiguration("top_map")
    prob_map = LaunchConfiguration("wire_prob_map")
    mode = LaunchConfiguration("mode")

    db_connection_arg = DeclareLaunchArgument(
        "db_connection_string", default_value="localhost:27017"
    )
    db_name_arg = DeclareLaunchArgument("db_name", default_value="refine-plan")
    db_collection_arg = DeclareLaunchArgument("db_collection")
    top_map_arg = DeclareLaunchArgument("top_map", default_value=top_map_path)
    prob_map_arg = DeclareLaunchArgument("wire_prob_map", default_value=prob_map_path)
    mode_arg = DeclareLaunchArgument("mode")

    policy_exec_node = Node(
        package="turtlebot_house_sim",
        executable="policy_executor",
        name="policy_executor",
        parameters=[
            {
                "db_connection_string": db_connection_string,
                "db_name": db_name,
                "db_collection": db_collection,
                "top_map": top_map,
                "wire_prob_map": prob_map,
                "mode": mode,
            }
        ],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(set_sim_time)
    ld.add_action(db_connection_arg)
    ld.add_action(db_name_arg)
    ld.add_action(db_collection_arg)
    ld.add_action(top_map_arg)
    ld.add_action(prob_map_arg)
    ld.add_action(mode_arg)
    ld.add_action(policy_exec_node)

    return ld
