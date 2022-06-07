#!/usr/bin/env python3

import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data

from usr_msgs.msg import LocationMsg

from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix


class Location(Node):
    def __init__(self) -> None:
        """!
        Object class constructor
        """

        super().__init__("location_node")

        # Allow callbacks to be executed in parallel without restriction.
        self.callback_group = ReentrantCallbackGroup()

        # ---------------------------------------------------------------------
        # Subscribers

        self.sub_rpm = self.create_subscription(
            msg_type=Imu,
            topic="/imu/data",
            callback=self.cb_imu_data,
            qos_profile=qos_profile_sensor_data,
            callback_group=self.callback_group,
        )
        self.sub_rpm = self.create_subscription(
            msg_type=NavSatFix,
            topic="/wifi_geo/fix",
            callback=self.cb_wifi_geo,
            qos_profile=qos_profile_sensor_data,
            callback_group=self.callback_group,
        )
        self.sub_rpm = self.create_subscription(
            msg_type=NavSatFix,
            topic="/fix",
            callback=self.cb_wifi_geo,
            qos_profile=qos_profile_sensor_data,
            callback_group=self.callback_group,
        )
        self.sub_velocity_cmd = self.create_publisher(
            msg_type=NavSatFix,
            topic="/custom_gps",
            qos_profile=qos_profile_sensor_data,
            callback_group=self.callback_group,
        )

    def spin_node(self) -> None:
        """!
        Function to spin the node
        """
        rclpy.spin(self)


# =============================================================================
def main(args=None) -> None:
    """!
    Location Node's Main
    """
    rclpy.init(args=args)

    location_node = Location()

    node_thread = threading.Thread(target=location_node.spin_node)
    node_thread.start()

    plotter_node.destroy_node()
    rclpy.shutdown()


# =============================================================================
if __name__ == "__main__":
    main()
# =============================================================================
