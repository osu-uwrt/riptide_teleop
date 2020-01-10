#!/usr/bin/env python

import rospy
from riptide_msgs.msg import LinearCommand, AttitudeCommand, DepthCommand, ResetControls, Imu, Depth

from pynput.keyboard import Key, Listener, KeyCode

class KeyboardTeleop():

    keys = {}
    rollSpeed = 15
    pitchSpeed = 15
    yawSpeed = 15
    xSpeed = 20
    ySpeed = 20
    depthSpeed = 0.2
    period = 0.1

    def __init__(self):
        self.xPub = rospy.Publisher("/command/x", LinearCommand, queue_size=10)
        self.yPub = rospy.Publisher("/command/y", LinearCommand, queue_size=10)
        self.rollPub = rospy.Publisher("/command/roll", AttitudeCommand, queue_size=10)
        self.pitchPub = rospy.Publisher("/command/pitch", AttitudeCommand, queue_size=10)
        self.yawPub = rospy.Publisher("/command/yaw", AttitudeCommand, queue_size=10)
        self.depthPub = rospy.Publisher("/command/depth", DepthCommand, queue_size=10)
        self.resetPub = rospy.Publisher("/controls/reset", ResetControls, queue_size=10)
        self.stop()

    def restrictAngle(self, angle):
        return (angle + 180) % 360 - 180
    
    def start(self):
        self.resetPub.publish(False)
        self.roll = 0
        self.pitch = 0
        self.yaw = rospy.wait_for_message("/state/imu", Imu).rpy_deg.z
        self.depth = max(rospy.wait_for_message("/state/depth", Depth).depth, 1)
        self.enabled = True
        rospy.loginfo("Keyboard Teleop enabled")

    def stop(self):
        self.enabled = False
        rospy.loginfo("Keyboard Teleop disabled. Press e to enable")

    def on_press(self, key):
        self.keys[key] = True

    def on_release(self, key):
        self.keys[key] = False

    def tick(self, event):
        if self.enabled:
            x = 0
            y = 0
            if Key.down in self.keys and self.keys[Key.down]:
                x -= self.xSpeed
            if Key.up in self.keys and self.keys[Key.up]:
                x += self.xSpeed
            if Key.right in self.keys and self.keys[Key.right]:
                self.yaw += self.yawSpeed * self.period
            if Key.left in self.keys and self.keys[Key.left]:
                self.yaw -= self.yawSpeed * self.period
            if KeyCode.from_char("w") in self.keys and self.keys[KeyCode.from_char("w")]:
                self.depth -= self.depthSpeed * self.period
            if KeyCode.from_char("s") in self.keys and self.keys[KeyCode.from_char("s")]:
                self.depth += self.depthSpeed * self.period
            if KeyCode.from_char("a") in self.keys and self.keys[KeyCode.from_char("a")]:
                y -= self.ySpeed
            if KeyCode.from_char("d") in self.keys and self.keys[KeyCode.from_char("d")]:
                y += self.ySpeed
            if KeyCode.from_char("i") in self.keys and self.keys[KeyCode.from_char("i")]:
                self.pitch -= self.pitchSpeed * self.period
            if KeyCode.from_char("k") in self.keys and self.keys[KeyCode.from_char("k")]:
                self.pitch += self.pitchSpeed * self.period
            if KeyCode.from_char("j") in self.keys and self.keys[KeyCode.from_char("j")]:
                self.roll -= self.rollSpeed * self.period
            if KeyCode.from_char("l") in self.keys and self.keys[KeyCode.from_char("l")]:
                self.roll += self.rollSpeed * self.period
            if KeyCode.from_char("0") in self.keys and self.keys[KeyCode.from_char("0")]:
                self.roll = 0
                self.pitch = 0
            if KeyCode.from_char("p") in self.keys and self.keys[KeyCode.from_char("p")]:
                self.stop()
            if KeyCode.from_char("x") in self.keys and self.keys[KeyCode.from_char("x")]:
                self.resetPub.publish(True)
                self.stop()

            self.roll = self.restrictAngle(self.roll)
            self.pitch = self.restrictAngle(self.pitch)
            self.yaw = self.restrictAngle(self.yaw)

            self.xPub.publish(x, LinearCommand.FORCE)
            self.yPub.publish(y, LinearCommand.FORCE)
            self.depthPub.publish(True, self.depth)
            self.rollPub.publish(self.roll, AttitudeCommand.POSITION)
            self.pitchPub.publish(self.pitch, AttitudeCommand.POSITION)
            self.yawPub.publish(self.yaw, AttitudeCommand.POSITION)
        else:
            if KeyCode.from_char("e") in self.keys and self.keys[KeyCode.from_char("e")]:
                self.start()


if __name__=="__main__":
    
    rospy.init_node('keyboard_teleop')

    print("")
    print("w,a,s,d\t\t\t-> yz linear movement")
    print("up,down,left,right\t-> yaw and x")
    print("i,j,k,l\t\t\t-> roll and pitch")
    print("")

    keyboardTeleop = KeyboardTeleop()
    rospy.Timer(rospy.Duration(keyboardTeleop.period), keyboardTeleop.tick)
    with Listener(
        on_press=keyboardTeleop.on_press,
        on_release=keyboardTeleop.on_release) as listener:

        rospy.spin()
