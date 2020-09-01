#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Joy
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Empty

from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
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

def curve(val):
    return val * abs(val)

class PS3Teleop():

    def __init__(self):
        config_path = rospy.get_param("~vehicle_config")
        with open(config_path, 'r') as stream:
            config = yaml.safe_load(stream)
            self.max_linear_velocity = config["maximum_linear_velocity"]
            self.max_angular_velocity = config["maximum_angular_velocity"]

        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_cb)
        self.odom_sub = rospy.Subscriber("odometry/filtered", Odometry, self.odom_cb)
        self.lin_vel_pub = rospy.Publisher("linear_velocity", Vector3, queue_size=1)
        self.orientation_pub = rospy.Publisher("orientation", Quaternion, queue_size=1)
        self.position_pub = rospy.Publisher("position", Vector3, queue_size=1)
        self.off_pub = rospy.Publisher("off", Empty, queue_size=1)

        self.START_DEPTH = -1
        self.last_odom_msg = rospy.get_rostime()
        self.last_linear_velocity = np.zeros(3)
        self.desired_orientation = np.array([0, 0, 0, 1.0])
        self.ang_vel = np.zeros(3)
        self.enabled = False

    def stop(self):
        linear_velocity = Vector3()
        self.lin_vel_pub.publish(linear_velocity)
        self.last_linear_velocity = linear_velocity
        self.enabled = False

    def joy_cb(self, msg):
        if msg.buttons[BUTTON_SHAPE_X]:
            self.stop()
            self.off_pub.publish()
            return

        if self.enabled and msg.buttons[BUTTON_SHAPE_SQUARE]:
            self.stop()
            return

        if self.enabled:
            linear_velocity = Vector3()
            linear_velocity.x = curve(msg.axes[AXES_STICK_RIGHT_UD]) * self.max_linear_velocity[0]
            linear_velocity.y = curve(msg.axes[AXES_STICK_LEFT_LR]) * self.max_linear_velocity[1]
            linear_velocity.z = curve(msg.axes[AXES_STICK_LEFT_UD]) * self.max_linear_velocity[2]

            self.ang_vel = np.zeros(3)
            self.ang_vel[2] = curve(msg.axes[AXES_STICK_RIGHT_LR]) * self.max_angular_velocity[2]
            if msg.buttons[BUTTON_CROSS_UP]:
                self.ang_vel[1] = self.max_angular_velocity[1]
            if msg.buttons[BUTTON_CROSS_DOWN]:
                self.ang_vel[1] = -self.max_angular_velocity[1]
            if msg.buttons[BUTTON_CROSS_RIGHT]:
                self.ang_vel[0] = self.max_angular_velocity[0]
            if msg.buttons[BUTTON_CROSS_LEFT]:
                self.ang_vel[0] = -self.max_angular_velocity[0]

            if not np.array_equal(self.last_linear_velocity, msgToNumpy(linear_velocity)):
                self.lin_vel_pub.publish(linear_velocity)
                self.last_linear_velocity = msgToNumpy(linear_velocity)

            if msg.buttons[BUTTON_SHAPE_CIRCLE]:
                r, p, y = euler_from_quaternion(self.desired_orientation)
                r, p = 0, 0
                self.desired_orientation = quaternion_from_euler(r, p, y)
        else:
            if msg.buttons[BUTTON_START]:
                odom_msg = rospy.wait_for_message("odometry/filtered", Odometry)
                r, p, y = euler_from_quaternion(msgToNumpy(odom_msg.pose.pose.orientation))
                r, p = 0, 0
                self.desired_orientation = quaternion_from_euler(r, p, y)
                desired_position = msgToNumpy(odom_msg.pose.pose.position)
                if desired_position[2] > self.START_DEPTH:
                    desired_position[2] = self.START_DEPTH
                self.position_pub.publish(*desired_position)
                self.enabled = True

    def odom_cb(self, msg):
        if self.enabled:
            dt = (rospy.get_rostime() - self.last_odom_msg).to_sec()
            rotation = quaternion_from_euler(*(self.ang_vel * dt))
            self.desired_orientation = quaternion_multiply(self.desired_orientation, rotation)
            self.orientation_pub.publish(*self.desired_orientation)

        self.last_odom_msg = rospy.get_rostime()


        


if __name__=="__main__":
    
    rospy.init_node('ps3_teleop')
    ps3_teleop = PS3Teleop()
    rospy.spin()
