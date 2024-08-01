#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
import os


def callback(data):
    if data.data != -1:
        os.system("/usr/local/bin/gpio write 25 1")
        rospy.sleep(1)
        os.system("/usr/local/bin/gpio write 25 0")


if __name__ == "__main__":
    rospy.init_node("led_node")
    pub = rospy.Subscriber("laser_and_led_order", Int32, callback)

    os.system("/usr/local/bin/gpio mode 25 out")
    os.system("/usr/local/bin/gpio write 25 0")

    rospy.loginfo("LED node started.")
    rospy.spin()
    os.system("/usr/local/bin/gpio mode 25 in")
