#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Empty

from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
import numpy as np
import math
import yaml

from pynput.keyboard import Key, Listener, KeyCode

def msgToNumpy(msg):
    if hasattr(msg, "w"):
        return np.array([msg.x, msg.y, msg.z, msg.w])
    return np.array([msg.x, msg.y, msg.z])

class KeyboardTeleop():

    def __init__(self):
        config_path = rospy.get_param("~vehicle_config")
        with open(config_path, 'r') as stream:
            config = yaml.safe_load(stream)
            self.max_linear_velocity = config["maximum_linear_velocity"]
            self.max_angular_velocity = config["maximum_angular_velocity"]

        self.odom_sub = rospy.Subscriber("odometry/filtered", Odometry, self.odom_cb)
        self.lin_vel_pub = rospy.Publisher("linear_velocity", Vector3, queue_size=10)
        self.orientation_pub = rospy.Publisher("orientation", Quaternion, queue_size=10)
        self.position_pub = rospy.Publisher("position", Vector3, queue_size=10)
        self.off_pub = rospy.Publisher("off", Empty, queue_size=10)

        self.START_DEPTH = -1
        self.last_odom_msg = rospy.get_rostime()
        self.last_linear_velocity = np.zeros(3)
        self.desired_orientation = np.array([0, 0, 0, 1.0])
        self.enabled = False
        self.keys = {}

    
    def start(self):
        odom_msg = rospy.wait_for_message("odometry/filtered", Odometry)
        r, p, y = euler_from_quaternion(msgToNumpy(odom_msg.pose.pose.orientation))
        r, p = 0, 0
        self.desired_orientation = quaternion_from_euler(r, p, y)
        desired_position = msgToNumpy(odom_msg.pose.pose.position)
        if desired_position[2] > self.START_DEPTH:
            desired_position[2] = self.START_DEPTH
        self.position_pub.publish(*desired_position)
        self.enabled = True
        rospy.loginfo("Keyboard Teleop enabled")

    def stop(self):
        linear_velocity = Vector3()
        self.lin_vel_pub.publish(linear_velocity)
        self.last_linear_velocity = linear_velocity
        self.enabled = False
        rospy.loginfo("Keyboard Teleop disabled. Press e to enable")

    def on_press(self, key):
        self.keys[key] = True
        if self.enabled:
            if key == KeyCode.from_char("p"):
                self.stop()
            if key == KeyCode.from_char("x"):
                self.stop()
                self.off_pub.publish()
            if key == KeyCode.from_char("0"):
                r, p, y = euler_from_quaternion(self.desired_orientation)
                r, p = 0, 0
                self.desired_orientation = quaternion_from_euler(r, p, y)
        else:
            if key == KeyCode.from_char("e"):
                self.start()

    def on_release(self, key):
        self.keys[key] = False

    def odom_cb(self, msg):
        if self.enabled:
            linear_velocity = Vector3()
            ang_vel = np.zeros(3)

            if Key.down in self.keys and self.keys[Key.down]:
                linear_velocity.x = -self.max_linear_velocity[0]
            if Key.up in self.keys and self.keys[Key.up]:
                linear_velocity.x = self.max_linear_velocity[0]
            if Key.right in self.keys and self.keys[Key.right]:
                ang_vel[2] = -self.max_angular_velocity[2]
            if Key.left in self.keys and self.keys[Key.left]:
                ang_vel[2] = self.max_angular_velocity[2]
            if KeyCode.from_char("w") in self.keys and self.keys[KeyCode.from_char("w")]:
                linear_velocity.z = self.max_linear_velocity[2]
            if KeyCode.from_char("s") in self.keys and self.keys[KeyCode.from_char("s")]:
                linear_velocity.z = -self.max_linear_velocity[2]
            if KeyCode.from_char("a") in self.keys and self.keys[KeyCode.from_char("a")]:
                linear_velocity.y = self.max_linear_velocity[1]
            if KeyCode.from_char("d") in self.keys and self.keys[KeyCode.from_char("d")]:
                linear_velocity.y = -self.max_linear_velocity[1]
            if KeyCode.from_char("i") in self.keys and self.keys[KeyCode.from_char("i")]:
                ang_vel[1] = self.max_angular_velocity[1]
            if KeyCode.from_char("k") in self.keys and self.keys[KeyCode.from_char("k")]:
                ang_vel[1] = -self.max_angular_velocity[1]
            if KeyCode.from_char("j") in self.keys and self.keys[KeyCode.from_char("j")]:
                ang_vel[0] = -self.max_angular_velocity[0]
            if KeyCode.from_char("l") in self.keys and self.keys[KeyCode.from_char("l")]:
                ang_vel[0] = self.max_angular_velocity[0]

            dt = (rospy.get_rostime() - self.last_odom_msg).to_sec()
            rotation = quaternion_from_euler(*(ang_vel * dt))
            self.desired_orientation = quaternion_multiply(self.desired_orientation, rotation)
            self.orientation_pub.publish(*self.desired_orientation)

            if not np.array_equal(self.last_linear_velocity, msgToNumpy(linear_velocity)):
                self.lin_vel_pub.publish(linear_velocity)
                self.last_linear_velocity = msgToNumpy(linear_velocity)

        self.last_odom_msg = rospy.get_rostime()
        


if __name__=="__main__":
    
    rospy.init_node('keyboard_teleop')

    print("p to pause, x to kill")
    print("")
    print("w,a,s,d\t\t\t-> y & z linear movement")
    print("up,down,left,right\t-> angular z and linear x")
    print("i,j,k,l\t\t\t-> y & z angular movement")
    print("")
    print("Keyboard Teleop disabled. Press e to enable")

    keyboardTeleop = KeyboardTeleop()
    with Listener(
        on_press=keyboardTeleop.on_press,
        on_release=keyboardTeleop.on_release) as listener:

        rospy.spin()
