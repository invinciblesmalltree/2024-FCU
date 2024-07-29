#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
import serial

def main():
    rospy.init_node('barcode_scanner_node')
    pub = rospy.Publisher('barcode_data', Int32, queue_size=10)
    rate = rospy.Rate(10)

    port = '/dev/ttyACM1'  # 串口设备路径
    baudrate = 9600
    timeout = 1
    ser = serial.Serial(port, baudrate, timeout=timeout)

    rospy.loginfo("Barcode reader node started.")

    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            data = ser.readline().strip()
            barcode = int(data)
            rospy.loginfo("Barcode scanned: %d" % barcode)
            pub.publish(barcode)
        else:
            pub.publish(-1)

        rate.sleep()

    ser.close()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass