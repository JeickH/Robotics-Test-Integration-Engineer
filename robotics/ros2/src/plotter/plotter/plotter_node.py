#!/usr/bin/env python3
# =============================================================================
"""! @package plotter_node.py

Code Information:
    Maintainer: Eng. Davidson Daniel Rojas Cediel
	Mail: davidson@kiwibot.com
	Kiwi Campus / AI & Robotics Team
"""

# =============================================================================

# Basics
import threading

# ROS2 dependencies
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

# ROS2 messages
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

# ROS2 Custom Messages
from usr_msgs.msg import MotorsRPM

from std_msgs.msg import Int8


# Plotter
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class Plotter(Node):
    def __init__(self) -> None:
        """!
        Object class constructor
        """

        super().__init__("plotter_node")

        # Define limits to play sounds
        self.upper_rpm_limit = 160
        self.lower_rpm_limit = -160
        self.sound_msg = Int8()

        # =============================================================================
        # Define Plotter Variables
        # =============================================================================
        self.fig, self.ax = plt.subplots(1, 3)
        # Extrapoint
        self.csfont = {
            "fontname": "DejaVu Sans Mono",
            "fontsize": 17,
            "color": "#003cc6",
        }
        self.cbfont = {
            "fontname": "DejaVu Sans Mono",
            "fontsize": 22,
            "color": "#003cc6",
        }
        self.axfont = {
            "fontname": "DejaVu Sans Mono",
            "fontsize": 10,
            "color": "#003cc6",
        }
        self.lgfont = {
            "fontsize": 12,
            "facecolor": "#f5cd74",
            "edgecolor": "#003cc6",
        }
        self.fig.suptitle("Amazing Plotter", **self.cbfont)
        self.fig.set_size_inches(18.5, 10.5)
        # =============================================================================
        # Controller Lines
        # =============================================================================

        # Linear
        (self.control_lin_ln,) = self.ax[0].plot(
            [],
            [],
            "#003cc6",
            label="Control Linear Signal",
            linewidth=2,
        )
        (self.error_linear_ln,) = self.ax[0].plot(
            [],
            [],
            "#00f0ff",
            label="Linear Error",
            linewidth=2,
        )
        self.controller_lin_lns = [self.control_lin_ln, self.error_linear_ln]
        self.ax[0].legend(**self.lgfont)
        self.x_linear_data, self.y_linear_data = [[], []], [[], []]

        # Angular

        # ---------------------------------------------------------------------
        # TODO: Initialize the second subplot
        # Take care about the variables.
        # NOT define new variables use the variables defined along the code
        #
        # Self-contained reference :smile:
        #
        # End Code
        # ---------------------------------------------------------------------

        (self.control_ang_ln,) = self.ax[1].plot(
            [], [], "#003cc6", label="Control Angular Signal", linewidth=2
        )
        (self.error_ang_ln,) = self.ax[1].plot(
            [], [], "#00f0ff", label="Angular Error", linewidth=2
        )
        self.controller_ang_lns = [self.control_ang_ln, self.error_ang_ln]
        self.ax[1].legend(**self.lgfont)
        self.x_ang_data, self.y_ang_data = [[], []], [[], []]

        (self.rpm_motor1_ln,) = self.ax[2].plot(
            [], [], "r", label="RPM Frontal Right Motor", linewidth=2
        )
        (self.rpm_motor2_ln,) = self.ax[2].plot(
            [], [], "b", label="RPM Rear Right Motor", linewidth=2
        )
        (self.rpm_motor3_ln,) = self.ax[2].plot(
            [], [], "m", label="RPM Rear Left Motor", linewidth=2
        )
        (self.rpm_motor4_ln,) = self.ax[2].plot(
            [], [], "g", label="RPM Frontal Left Motor", linewidth=2
        )
        self.rpm_motors_lns = [
            self.rpm_motor1_ln,
            self.rpm_motor2_ln,
            self.rpm_motor3_ln,
            self.rpm_motor4_ln,
        ]
        self.ax[2].legend(**self.lgfont)
        self.x_rpm_data, self.y_rpm_data = [[], [], [], []], [[], [], [], []]
        # Extra point
        self.axes_curr_limits = {
            "sp1": {"xlim": [0, 500], "ylim": [-3, 3]},
            "sp2": {"xlim": [0, 500], "ylim": [-3, 3]},
            "sp1": {"xlim": [0, 500], "ylim": [-170, 170]},
        }

        # =============================================================================
        # ROS2 Stuffs
        # =============================================================================
        # Allow callbacks to be executed in parallel without restriction.
        self.callback_group = ReentrantCallbackGroup()

        # ---------------------------------------------------------------------
        # Subscribers

        self.sub_velocity_cmd = self.create_subscription(
            msg_type=Twist,
            topic="/motion_control/speed_controller/output_cmd",
            callback=self.cb_cmd_vel,
            qos_profile=5,
            callback_group=self.callback_group,
        )

        self.sub_velocity_cmd = self.create_subscription(
            msg_type=TwistStamped,
            topic="/motion_control/speed_controller/error",
            callback=self.cb_error_vel,
            qos_profile=5,
            callback_group=self.callback_group,
        )

        self.sub_rpm = self.create_subscription(
            msg_type=MotorsRPM,
            topic="/uavcan/chassis/motors_rpm_feedback",
            callback=self.cb_rpm_feedback,
            qos_profile=5,
            callback_group=self.callback_group,
        )

        # Publisher
        self.pub_sound = self.create_publisher(
            msg_type=Int8,
            topic="/device/speaker/command",
            qos_profile=5,
            callback_group=self.callback_group,
        )

    # Hack Function to not block the thread
    def spin_node(self) -> None:
        """!
        Function to spin the node
        """
        rclpy.spin(self)

    # Plotter Functions
    def plot_init(self) -> None:
        """!
        Function to set the initial plot status.
        """
        # Extrapoint

        self.fig.patch.set_facecolor("#59c9ff")
        # self.control_lin_ln.set_color("#f5cd74")
        # self.error_linear_ln.set_color("#f5cd74")
        # self.ax[0].yaxis.label.set_color("#f5cd74")
        # self.fig.

        self.ax[0].set_xlim(0, 500)
        self.ax[0].set_ylim(-3, 3)
        self.ax[0].set_title("Linear Signal / Linear Error", **self.csfont)

        self.ax[1].set_xlim(0, 500)
        self.ax[1].set_ylim(-3, 3)
        self.ax[1].set_title("Angular Signal / Angular Error", **self.csfont)

        self.ax[2].set_xlim(0, 500)
        self.ax[2].set_ylim(-170, 170)
        self.ax[2].set_title("RPMs", **self.csfont)

        # return [self.controller_lin_lns, self.controller_ang_lns]
        return [self.controller_lin_lns, self.controller_ang_lns, self.rpm_motors_lns]

    def update_plot(self, frame=None) -> None:
        """!
        Function to update the current figure
        """
        self.controller_lin_lns[0].set_data(
            self.x_linear_data[0], self.y_linear_data[0]
        )
        self.controller_lin_lns[1].set_data(
            self.x_linear_data[1], self.y_linear_data[1]
        )
        self.controller_ang_lns[0].set_data(self.x_ang_data[0], self.y_ang_data[0])
        self.controller_ang_lns[1].set_data(self.x_ang_data[1], self.y_ang_data[1])

        # Update rpm values
        self.rpm_motors_lns[0].set_data(self.x_rpm_data[0], self.y_rpm_data[0])
        self.rpm_motors_lns[1].set_data(self.x_rpm_data[1], self.y_rpm_data[1])
        self.rpm_motors_lns[2].set_data(self.x_rpm_data[2], self.y_rpm_data[2])
        self.rpm_motors_lns[3].set_data(self.x_rpm_data[3], self.y_rpm_data[3])

        # size the subplot dynamically
        if len(self.x_linear_data[0]) > 500:
            self.ax[0].set_xlim(
                len(self.x_linear_data[0]) - 500, len(self.x_linear_data[0])
            )
        if len(self.x_ang_data[0]) > 500:
            self.ax[1].set_xlim(len(self.x_ang_data[0]) - 500, len(self.x_ang_data[0]))
        if len(self.x_rpm_data[0]) > 500:
            self.ax[2].set_xlim(len(self.x_rpm_data[0]) - 500, len(self.x_rpm_data[0]))

        return [self.controller_lin_lns, self.controller_ang_lns, self.rpm_motors_lns]

    # Callback functions
    def cb_cmd_vel(self, msg: Twist) -> None:
        """!
        Callback function to get the velocity control signal.
        @param msg 'Twist' message containing the velocities of the robot
        """
        self.y_linear_data[0].append(msg.linear.x)
        x_index = len(self.x_linear_data[0])
        self.x_linear_data[0].append(x_index + 1)

        self.y_ang_data[0].append(msg.angular.z)
        x_index2 = len(self.x_ang_data[0])
        self.x_ang_data[0].append(x_index2 + 1)

    def cb_error_vel(self, msg: TwistStamped) -> None:
        """!
        Callback function to get the error between the reference and the current velocity
        @param msg 'TwistStamped' message containing the velocities of the robot
        """

        self.y_linear_data[1].append(msg.twist.linear.x)
        x_index = len(self.x_linear_data[1])
        self.x_linear_data[1].append(x_index + 1)

        self.y_ang_data[1].append(msg.twist.angular.z)
        x_index2 = len(self.x_ang_data[1])
        self.x_ang_data[1].append(x_index2 + 1)

    def cb_rpm_feedback(self, msg: MotorsRPM) -> None:
        """!
        Callback function to get motors RPMS feedback
        @param msg 'MotorsRPM' message containing the velocities of the robot
        """
        self.y_rpm_data[0].append(msg.rpms_fr)
        x_index = len(self.x_rpm_data[0])
        self.x_rpm_data[0].append(x_index + 1)

        self.y_rpm_data[1].append(msg.rpms_rr)
        x_index = len(self.x_rpm_data[1])
        self.x_rpm_data[1].append(x_index + 1)

        self.y_rpm_data[2].append(msg.rpms_rl)
        x_index = len(self.x_rpm_data[2])
        self.x_rpm_data[2].append(x_index + 1)

        self.y_rpm_data[3].append(msg.rpms_fl)
        x_index = len(self.x_rpm_data[3])
        self.x_rpm_data[3].append(x_index + 1)

        # Extra homework sounds when rpm increase or decrease of X value
        if (
            msg.rpms_fr > self.upper_rpm_limit
            or msg.rpms_rr > self.upper_rpm_limit
            or msg.rpms_rl > self.upper_rpm_limit
            or msg.rpms_fl > self.upper_rpm_limit
        ):
            print(f"rpms upper than limit")
            self.sound_msg.data = 3
            self.pub_sound.publish(self.sound_msg)

        if (
            msg.rpms_fr < self.lower_rpm_limit
            or msg.rpms_rr < self.lower_rpm_limit
            or msg.rpms_rl < self.lower_rpm_limit
            or msg.rpms_fl < self.lower_rpm_limit
        ):
            print(f"rpms lower than limit")
            self.sound_msg.data = 0
            self.pub_sound.publish(self.sound_msg)

        return


# =============================================================================
def main(args=None) -> None:
    """!
    Plotter Node's Main
    """

    # Initialize ROS communications for a given context.
    rclpy.init(args=args)

    # Execute work and block until the context associated with the
    # executor is shutdown.
    plotter_node = Plotter()

    # ---------------------------------------------------------------------
    # TODO: Create a Thread for spin the node
    # Use the function spin_node
    # https://realpython.com/intro-to-python-threading/
    #
    node_thread = threading.Thread(target=plotter_node.spin_node)
    node_thread.start()
    # End Code
    # ---------------------------------------------------------------------

    ani = FuncAnimation(
        plotter_node.fig,
        plotter_node.update_plot,
        init_func=plotter_node.plot_init,
    )

    plt.show(block=True)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    plotter_node.destroy_node()
    rclpy.shutdown()


# =============================================================================
if __name__ == "__main__":
    main()
# =============================================================================
