#!/usr/bin/env python3
""" Node which manages the wire in the house.

Author: Charlie Street
Owner: Charlie Street
"""

from topological_navigation.topological_map import TopologicalMap
from visualization_msgs.msg import Marker, MarkerArray
from wire_msgs.srv import WireCheck
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.node import Node
import numpy as np
import rclpy
import yaml


# Offset for RViz wire markers
MARKER_OFFSET = 7004


class WireManager(Node):
    """A node which manages a wire in the environment.

    The manager places a wire and allows a user to check if its at a location.

    Attributes:
        _wire_loc: The topological location of the wire
        _top_map: The topological map
        _marker_pub: A publisher for all markers in RViz
        _check_for_wire: A service which returns whether the wire is at a location
    """

    def __init__(self):
        super().__init__("wire_manager")

        self.declare_parameter("top_map", rclpy.Parameter.Type.STRING)
        self.declare_parameter("wire_prob_map_yaml", rclpy.Parameter.Type.STRING)
        self.declare_parameter("wire_status_file", rclpy.Parameter.Type.STRING)
        self.declare_parameter("wire_status_index", rclpy.Parameter.Type.INTEGER)

        self._top_map = TopologicalMap(self.get_parameter("top_map").value)

        self._initialise_wire()

        # Publish markers
        latching_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._marker_pub = self.create_publisher(
            MarkerArray, "wire_markers", qos_profile=latching_qos
        )

        self._publish_wire_markers()

        self._check_for_wire = self.create_service(
            WireCheck, "check_for_wire", self._wire_check_callback
        )

        self.get_logger().info("Wire Manager Started")

    def _initialise_wire(self):
        """Initialise the wire."""
        wire_status_file = self.get_parameter("wire_status_file").value
        wire_status_idx = self.get_parameter("wire_status_index").value

        # Read from file if both parameters set
        if wire_status_file != "not_set" and wire_status_file != -1:
            self.get_logger().info("Reading Wire Location From File.")
            with open(wire_status_file, "r") as yaml_in:
                self._wire_loc = yaml.load(yaml_in, Loader=yaml.FullLoader)[
                    wire_status_idx
                ]
        else:  # Sample from wire_prob_map_yaml
            self.get_logger().info("Sampling Wire Location.")
            prob_map_file = self.get_parameter("wire_prob_map_yaml").value
            with open(prob_map_file, "r") as yaml_in:
                prob_map = yaml.load(yaml_in, Loader=yaml.FullLoader)

            locs = [entry["node"] for entry in prob_map]
            probs = [entry["p_wire"] for entry in prob_map]
            self._wire_loc = np.random.choice(locs, p=probs)

        self.get_logger().info("Wire Located at: {}".format(self._wire_loc))

    def _publish_wire_markers(self):
        """Publish the markers showing where the wire is."""
        marker_array = MarkerArray()

        marker_id = 0

        # Get the possible locations for the wire and only visualise those
        prob_map_file = self.get_parameter("wire_prob_map_yaml").value
        with open(prob_map_file, "r") as yaml_in:
            prob_map = yaml.load(yaml_in, Loader=yaml.FullLoader)
            possible_nodes = [entry["node"] for entry in prob_map]

        for node in possible_nodes:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.id = MARKER_OFFSET + marker_id
            marker.type = 2  # Sphere
            marker.action = 0
            marker.pose.position.x = self._top_map._nodes[node].x
            marker.pose.position.y = self._top_map._nodes[node].y
            marker.pose.position.z = 0.5
            marker.scale.x = 0.4
            marker.scale.y = 0.4
            marker.scale.z = 0.4
            marker.color.r = 0.72 if self._wire_loc != node else 0.3
            marker.color.g = 0.11 if self._wire_loc != node else 0.69
            marker.color.b = 0.11 if self._wire_loc != node else 0.31
            marker.color.a = 0.9
            marker_id += 1
            marker_array.markers.append(marker)

        self._marker_pub.publish(marker_array)
        self.get_logger().info("Wire Markers Published.")

    def _wire_check_callback(self, request, response):
        """Check if the wire is at a given location.

        Args:
            request: Contains the node to check
            response: Contains a flag stating if the wire is present

        Returns:
            The filled in response object
        """
        self.get_logger().info("Wire Check at Node {}".format(request.node))
        if self._wire_loc == request.node:
            response.found = True
        else:
            response.found = False

        return response


def main(args=None):
    rclpy.init(args=args)

    wire_manager = WireManager()
    rclpy.spin(wire_manager)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
