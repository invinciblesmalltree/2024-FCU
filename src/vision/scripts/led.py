#!/usr/bin/env python

import os
import sys

if os.geteuid() != 0:
    os.execvp("sudo", ["sudo"] + ["python3"] + sys.argv)

import rospy
from std_msgs.msg import Int32
import OPi.GPIO as GPIO


def callback(data):
    if data.data != -1:
        GPIO.output(37, GPIO.HIGH)
        rospy.sleep(1)
        GPIO.output(37, GPIO.LOW)


if __name__ == "__main__":
    rospy.init_node("led_node")
    pub = rospy.Subscriber("laser_and_led_order", Int32, callback)

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(37, GPIO.OUT)
    GPIO.output(37, GPIO.LOW)

    rospy.loginfo("LED node started.")
    rospy.spin()
