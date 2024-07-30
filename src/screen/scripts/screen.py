# !/usr/bin/env python3

# 注：评委输入编号1-24，返回地址A1-D6

import rospy
import serial
from std_msgs.msg import Int32
import json


# json文件路径
json_file_path = 'goods_info.json'
address_list = ['A3', 'A2', 'A1', 'A4', 'A5', 'A6',
                'C6', 'C5', 'C4', 'C1', 'C2', 'C3',
                'B1', 'B2', 'B3', 'B6', 'B5', 'B4',
                'D4', 'D5', 'D6', 'D1', 'D2', 'D3']
index = 0

# 更新货物信息
def update_goods_data(file_path, address, value):
    with open(file_path, 'r') as json_file:
        data = json.load(json_file)
    if address in data[0]:
        data[0][address] = value
        with open(file_path, 'w') as json_file:
            json.dump(data, json_file, indent=4)
        rospy.loginfo(f"Updated value for address {address}: {value}")
    else:
        rospy.loginfo(f"Address {address} not found in JSON data.")

# 评委根据值查询地址
def get_address_by_value(file_path, value):
    with open(file_path, 'r') as json_file:
        data = json.load(json_file)
    for address, stored_value in data[0].items():
        if stored_value == value:
            return address
    return None

# 货物编号为1～24的数值；坐标信息为A1~A6、B1~B6、C1~C6、D1~D6
def goods_callback(msg):
    update_goods_data(json_file_path, address_list[index], msg.data)
    index += 1

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