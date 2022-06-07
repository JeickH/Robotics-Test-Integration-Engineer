#!/usr/bin/env python3

import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data

from usr_msgs.msg import LocationMsg

from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix

from math import asin, atan2


class Location(threading.Thread, Node):
    def __init__(self) -> None:
        """!
        Object class constructor
        """
        threading.Thread.__init__(self)
        Node.__init__(self, node_name="location_node")
        # Allow callbacks to be executed in parallel without restriction.
        self.callback_group = ReentrantCallbackGroup()
        # Define some variables
        self.roll = 0
        self.yaw = 0
        self.pitch = 0
        self.latitude: float = 0
        self.longitude: float = 0
        self.sensor: str = ""
        self.msg = LocationMsg()
        self.done = False

        # Subscribers
        self.sub_imu = self.create_subscription(
            msg_type=Imu,
            topic="/imu/data",
            callback=self.cb_imu_data,
            qos_profile=qos_profile_sensor_data,
            callback_group=self.callback_group,
        )
        self.sub_wifi_geo = self.create_subscription(
            msg_type=NavSatFix,
            topic="/wifi_geo/fix",
            callback=self.cb_wifi_geo,
            qos_profile=qos_profile_sensor_data,
            callback_group=self.callback_group,
        )
        self.sub_gps_fix = self.create_subscription(
            msg_type=NavSatFix,
            topic="/fix",
            callback=self.cb_gps_data,
            qos_profile=qos_profile_sensor_data,
            callback_group=self.callback_group,
        )
        # Publisher
        self.pub_custom_gps = self.create_publisher(
            msg_type=LocationMsg,
            topic="/custom_gps",
            qos_profile=qos_profile_sensor_data,
            callback_group=self.callback_group,
        )

        self.start()

    def run(self) -> None:
        while not self.done:
            self.msg.latitude = str(self.latitude)
            self.msg.longitude = str(self.longitude)
            self.msg.sensor = self.sensor
            self.msg.pitch = float(self.pitch)
            self.msg.roll = float(self.roll)
            self.msg.yaw = float(self.yaw)
            self.pub_custom_gps.publish(self.msg)

    def stop(self):
        self.done = True

    def cb_imu_data(self, msg: Imu):
        """!
        Callback function to get the imu data.
        @param msg 'Imu' message containing the imu data of the robot
        """
        self.done = False
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        self.yaw = atan2(
            2.0 * (qy * qz + qw * qx), qw * qw - qx * qx - qy * qy + qz * qz
        )
        self.pitch = asin(-2.0 * (qx * qz - qw * qy))
        self.roll = atan2(
            2.0 * (qx * qy + qw * qz), qw * qw + qx * qx - qy * qy - qz * qz
        )

    def cb_wifi_geo(self, msg: NavSatFix):
        """!
        Callback function to get the wifi geo localization.
        @param msg 'NavSatFix' message containing the localization data from the router gps
        """
        self.done = False
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.sensor = "WIFI"

    def cb_gps_data(self, msg: NavSatFix):
        """!
        Callback function to get the gps localization.
        @param msg 'NavSatFix' message containing the localization data from the gps sensor
        """
        self.done = False
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.sensor = "GPS"


def main(args=None) -> None:
    """!
    Location Node's Main
    """
    rclpy.init(args=args)

    location_node = Location()
    rclpy.spin(location_node)
    # node_thread = threading.Thread(target=location_node.spin_node)
    # node_thread.start()

    location_node.clear()
    location_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
# =============================================================================
