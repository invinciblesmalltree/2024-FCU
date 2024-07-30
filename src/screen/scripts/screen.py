#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import Int32

# 货物编号为1～24的数值；坐标信息为A1~A6、B1~B6、C1~C6、D1~D6
def goods_callback(msg):
    if msg.data != 1:
        goods_list.append(msg.data)
        rospy.loginfo("goods_data: %d" % msg.data)

# 配置串口
ser = serial.Serial("/dev/AMA0", baudrate=9600, timeout=1)
# 起飞发布，发1为盘点程序，发2为定向程序
pub = rospy.Publisher("/offboard_order", Int32, queue_size=10)
# 从scanner订阅货物信息
rospy.Subscriber("/barcode_data", Int32, goods_callback)


# 主程序
rospy.init_node("screen", anonymous=True)
ser.write(b"rest\xff\xff\xff") # 重置屏幕

# 创建货物列表
goods_list = []

while not rospy.is_shutdown():
    if ser.in_waiting > 0:
        try:
            line = ser.readline().decode("utf-8").strip() # 读取串口数据
            if line == "offboard1":
                rospy.loginfo("offboard cmd1 from serial")
                pub.publish(Int32(1))
            elif line == "offboard2":
                rospy.loginfo("offboard cmd2 from serial")
                pub.publish(Int32(2))
        except Exception as e:
            pass

    rospy.sleep(0.1)