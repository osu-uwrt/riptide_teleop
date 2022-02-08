#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Joy
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Empty
#TODO: remove tf.transformations
from transforms3d import quat2euler, euler2quat
import numpy as np
import math
import yaml

AXES_STICK_LEFT_LR = 0 # Fully Leftwards = +1, Mid = 0, Fully Rightwards = -1
AXES_STICK_LEFT_UD = 1 # Fully Upwards = +1, Mid = 0, Fully Downwards = -1
AXES_STICK_RIGHT_LR = 3 # Fully Leftwards = +1, Mid = 0, Fully Rightwards = -1
AXES_STICK_RIGHT_UD = 4 # Fully Upwards = +1, Mid = 0, Fully Downwards = -1
AXES_REAR_L2 = 2 # Released = +1, Mid = 0, Fully Pressed = -1
AXES_REAR_R2 = 5 # Released = +1, Mid = 0, Fully Pressed = -1

BUTTON_SHAPE_X = 0
BUTTON_SHAPE_CIRCLE = 1
BUTTON_SHAPE_TRIANGLE = 2
BUTTON_SHAPE_SQUARE = 3
BUTTON_REAR_L1 = 4
BUTTON_REAR_R1 = 5
BUTTON_REAR_L2 = 6
BUTTON_REAR_R2 = 7
BUTTON_SELECT = 8
BUTTON_START = 9
BUTTON_PAIRING = 10
BUTTON_STICK_LEFT = 11
BUTTON_STICK_RIGHT = 12
BUTTON_CROSS_UP = 13
BUTTON_CROSS_DOWN = 14
BUTTON_CROSS_LEFT = 15
BUTTON_CROSS_RIGHT = 16

def msgToNumpy(msg):
    if hasattr(msg, "w"):
        return np.array([msg.x, msg.y, msg.z, msg.w])
    return np.array([msg.x, msg.y, msg.z])

# Joystick curve to apply. This is a x^2 that keeps polarity
def curve(val):
    return val * abs(val)

class PS3Teleop(Node):

    def __init__(self):
        super().__init__('riptide_teleop2')
        config_path = rclpy.get_param("~vehicle_config")
        with open(config_path, 'r') as stream:
            config = yaml.safe_load(stream)
            self.max_linear_velocity = config["maximum_linear_velocity"]
            self.max_angular_velocity = config["maximum_angular_velocity"]

        self.START_DEPTH = -1
        self.last_odom_msg = self.get_clock().now()
        self.last_linear_velocity = np.zeros(3)
        self.desired_orientation = np.array([0, 0, 0, 1.0])
        self.ang_vel = np.zeros(3)
        self.enabled = False

        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_cb,  qos_profile_system_default)
        self.odom_sub = self.create_subscription(Odometry, "odometry/filtered", self.odom_cb, qos_profile_system_default)
        self.lin_vel_pub = self.create_publisher(Vector3, "linear_velocity", qos_profile_system_default)
        self.orientation_pub = self.create_publisher(Quaternion, "orientation", qos_profile_system_default)
        self.position_pub = self.create_publisher(Vector3, "position", qos_profile_system_default)
        self.off_pub = self.create_publisher(Empty, "off", qos_profile_system_default)
      
    def joy_cb(self, msg):
        # Kill button
        if msg.buttons[BUTTON_SHAPE_X]:
            self.enabled = False
            self.off_pub.publish()
            return

        # Pause button
        if self.enabled and msg.buttons[BUTTON_SHAPE_SQUARE]:
            self.enabled = False
            self.lin_vel_pub.publish(Vector3())
            self.last_linear_velocity = np.zeros(3)
            return

        # While controller is enabled
        if self.enabled:
            # Build linear velocity
            linear_velocity = Vector3()
            linear_velocity.x = curve(msg.axes[AXES_STICK_RIGHT_UD]) * self.max_linear_velocity[0]
            linear_velocity.y = curve(msg.axes[AXES_STICK_LEFT_LR]) * self.max_linear_velocity[1]
            linear_velocity.z = curve(msg.axes[AXES_STICK_LEFT_UD]) * self.max_linear_velocity[2]

            # Build angular velocity
            # 0.9 is to allow the robot to catch up to the moving target
            self.ang_vel = np.zeros(3)
            self.ang_vel[2] = curve(msg.axes[AXES_STICK_RIGHT_LR]) * self.max_angular_velocity[2] * 0.9
            if msg.buttons[BUTTON_CROSS_UP]:
                self.ang_vel[1] = self.max_angular_velocity[1] * 0.9
            if msg.buttons[BUTTON_CROSS_DOWN]:
                self.ang_vel[1] = -self.max_angular_velocity[1] * 0.9
            if msg.buttons[BUTTON_CROSS_RIGHT]:
                self.ang_vel[0] = self.max_angular_velocity[0] * 0.9
            if msg.buttons[BUTTON_CROSS_LEFT]:
                self.ang_vel[0] = -self.max_angular_velocity[0] * 0.9


            # Publish linear velocity if the joystick has been touched
            if not np.array_equal(self.last_linear_velocity, msgToNumpy(linear_velocity)):
                self.lin_vel_pub.publish(linear_velocity)
                self.last_linear_velocity = msgToNumpy(linear_velocity)

            # Zero roll and pitch
            if msg.buttons[BUTTON_SHAPE_CIRCLE]:
                r, p, y = quat2euler(self.desired_orientation)
                r, p = 0, 0
                self.desired_orientation = quat2euler(r, p, y)
        else:
            # Start controlling
            if msg.buttons[BUTTON_START]:
                # Zero roll and pitch
                odom_msg = rclpy.wait_for_message("odometry/filtered", Odometry)
                r, p, y = quat2euler(msgToNumpy(odom_msg.pose.pose.orientation))
                r, p = 0, 0
                self.desired_orientation = quaternion_from_euler(r, p, y)

                # Submerge if not submerged. Else stop the bot
                desired_position = msgToNumpy(odom_msg.pose.pose.position)
                if desired_position[2] > self.START_DEPTH:
                    desired_position[2] = self.START_DEPTH
                    self.position_pub.publish(*desired_position)
                else:
                    self.lin_vel_pub.publish(Vector3())

                self.enabled = True

    def odom_cb(self, msg):
        if self.enabled:
            dt = (self.get_clock().now() - self.last_odom_msg).to_sec()
            rotation = euler2quat(*(self.ang_vel * dt))
            self.desired_orientation = euler2quat(self.desired_orientation, rotation)
            self.orientation_pub.publish(*self.desired_orientation)

        self.last_odom_msg = self.get_clock().now()

if __name__=="__main__":
    
    rclpy.init_node('ps3_teleop')
    ps3_teleop = PS3Teleop()
    rclpy.spin()
