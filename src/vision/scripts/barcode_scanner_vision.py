#!/usr/bin/env python

import rospy
from pyzbar.pyzbar import decode
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv


def get_center_point(points):
    x_coords = [point.x for point in points]
    y_coords = [point.y for point in points]
    center_x = sum(x_coords) // len(points)
    center_y = sum(y_coords) // len(points)
    return (center_x, center_y)


def callback(data):
    global frame
    try:
        frame = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)

    try:
        barcodes = decode(frame)
    except Exception as e:
        rospy.logerr(f"Barcode decoding failed: {e}")
        barcode_data_pub.publish(-1)
        return

    if barcodes:
        image_center = (frame.shape[1] // 2, frame.shape[0] // 2)
        closest_barcode = None
        min_distance = float("inf")

        for barcode in barcodes:
            barcode_center = get_center_point(barcode.polygon)
            distance = (
                (barcode_center[0] - image_center[0]) ** 2
                + (barcode_center[1] - image_center[1]) ** 2
            ) ** 0.5

            if distance < min_distance:
                min_distance = distance
                closest_barcode = barcode

        if closest_barcode:
            barcode_data = closest_barcode.data.decode()
            barcode_center = get_center_point(closest_barcode.polygon)
            if not barcode_data.isdigit():
                barcode_data_pub.publish(-1)
                vert_deviation_pub.publish(0)
                return
            barcode_data = int(barcode_data)
            barcode_data_pub.publish(barcode_data)
            vert_deviation_pub.publish(
                (image_center[1] - barcode_center[1] - 100) / 1000
            )
            return

    barcode_data_pub.publish(-1)
    vert_deviation_pub.publish(0)


rospy.init_node("barcode", anonymous=True)
rgb_sub = rospy.Subscriber("/d435/rgb", Image, callback)
barcode_data_pub = rospy.Publisher("/barcode_data_vision", Int32, queue_size=10)
vert_deviation_pub = rospy.Publisher("/vert_deviation", Float32, queue_size=10)

bridge = CvBridge()
frame = None
rospy.spin()
