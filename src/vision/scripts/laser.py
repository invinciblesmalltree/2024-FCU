#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
import Jetson.GPIO as GPIO


def callback(data):
    GPIO.output(11, GPIO.HIGH)
    rospy.sleep(0.5)
    GPIO.output(11, GPIO.LOW)


if __name__ == "__main__":
    rospy.init_node("laser_node")
    pub = rospy.Subscriber("laser_data", Int32, callback)

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(11, GPIO.OUT)
    GPIO.output(11, GPIO.LOW)

    rospy.loginfo("Laser node started.")
    rospy.spin()
