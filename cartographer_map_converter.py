#!/usr/bin/env python3
"""
Cartographer Map Converter

Converts Cartographer's occupancy grid (probability values 0-255) to
standard ROS 3-value format (-1=unknown, 0=free, 100=obstacle).

Handles both full map (/cartographer_map_raw → /cmap) and incremental
updates (/cartographer_map_raw_updates → /cmap_updates).

Based on the solution from:
    https://github.com/hrnr/m-explore/issues/28
"""

import rospy
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate


class MapConverter:
    def __init__(self):
        self.obstacle_thresh = rospy.get_param("~obstacle_threshold", 75)
        self.free_thresh = rospy.get_param("~free_threshold", 50)

        output_topic = rospy.get_param("~output_topic", "/cmap")
        updates_topic = rospy.get_param("~updates_output_topic", "/cmap_updates")
        map_input = rospy.get_param("~map_input_topic", "/cartographer_map_raw")
        updates_input = rospy.get_param("~updates_input_topic",
                                        map_input + "_updates")

        # Full map
        self.map_pub = rospy.Publisher(
            output_topic, OccupancyGrid, queue_size=1, latch=True)
        self.map_sub = rospy.Subscriber(
            map_input, OccupancyGrid, self.map_callback, queue_size=1)

        # Incremental updates
        self.updates_pub = rospy.Publisher(
            updates_topic, OccupancyGridUpdate, queue_size=1, latch=True)
        self.updates_sub = rospy.Subscriber(
            updates_input, OccupancyGridUpdate, self.updates_callback, queue_size=1)

        rospy.loginfo(
            "Cartographer map converter ready. "
            "Thresholds: obstacle>=%d, free<%d  map: %s→%s  updates: %s→%s",
            self.obstacle_thresh, self.free_thresh,
            map_input, output_topic, updates_input, updates_topic,
        )

    # ------------------------------------------------------------------
    # Shared conversion logic
    # ------------------------------------------------------------------

    def convert_cell(self, val):
        """Convert a single Cartographer probability value to standard 3-value."""
        if val < 0:
            return -1          # unknown
        elif val >= self.obstacle_thresh:
            return 100         # obstacle
        elif val >= 0 and val < self.free_thresh:
            return 0           # free
        else:
            return -1          # uncertain → unknown

    def convert_data(self, data, height, width):
        """Convert an entire grid (Cartographer row-major, origin bottom-left)."""
        data = list(data)
        for y in range(height):
            for x in range(width):
                i = x + (height - 1 - y) * width
                data[i] = self.convert_cell(data[i])
        return data

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def map_callback(self, msg):
        msg.data = tuple(self.convert_data(msg.data, msg.info.height, msg.info.width))
        self.map_pub.publish(msg)
        rospy.logdebug("Converted full map: %dx%d", msg.info.width, msg.info.height)

    def updates_callback(self, msg):
        msg.data = tuple(self.convert_data(msg.data, msg.height, msg.width))
        self.updates_pub.publish(msg)
        rospy.logdebug("Converted map update: %dx%d at (%d,%d)",
                       msg.width, msg.height, msg.x, msg.y)


if __name__ == "__main__":
    rospy.init_node("cartographer_map_converter")
    node = MapConverter()
    rospy.spin()
