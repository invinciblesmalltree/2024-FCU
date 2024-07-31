# !/usr/bin/env python3
import rospy
import serial
from std_msgs.msg import Int32
from trx_screen.msg import goods_info, coordinate_info
import json

json_data = {
    "A1": {"value": -1, "coordinate": [0.0, 1.75, 1.25, 0]},
    "A2": {"value": -1, "coordinate": [0.0, 1.25, 1.25, 0]},
    "A3": {"value": -1, "coordinate": [0.0, 0.75, 1.25, 0]},
    "A4": {"value": -1, "coordinate": [0.0, 1.75, 0.85, 0]},
    "A5": {"value": -1, "coordinate": [0.0, 1.25, 0.85, 0]},
    "A6": {"value": -1, "coordinate": [0.0, 0.75, 0.85, 0]},
    "B1": {"value": -1, "coordinate": [1.75, 0.75, 1.25, 3.14159]},
    "B2": {"value": -1, "coordinate": [1.75, 1.25, 1.25, 3.14159]},
    "B3": {"value": -1, "coordinate": [1.75, 1.75, 1.25, 3.14159]},
    "B4": {"value": -1, "coordinate": [1.75, 0.75, 0.85, 3.14159]},
    "B5": {"value": -1, "coordinate": [1.75, 1.25, 0.85, 3.14159]},
    "B6": {"value": -1, "coordinate": [1.75, 0.75, 0.85, 3.14159]},
    "C1": {"value": -1, "coordinate": [1.75, 1.75, 1.25, 0]},
    "C2": {"value": -1, "coordinate": [1.75, 1.25, 1.25, 0]},
    "C3": {"value": -1, "coordinate": [1.75, 0.75, 1.25, 0]},
    "C4": {"value": -1, "coordinate": [1.75, 1.75, 0.85, 0]},
    "C5": {"value": -1, "coordinate": [1.75, 1.25, 0.85, 0]},
    "C6": {"value": -1, "coordinate": [1.75, 0.75, 0.85, 0]},
    "D1": {"value": -1, "coordinate": [3.50, 1.75, 1.25, 3.14159]},
    "D2": {"value": -1, "coordinate": [3.50, 1.25, 1.25, 3.14159]},
    "D3": {"value": -1, "coordinate": [3.50, 0.75, 1.25, 3.14159]},
    "D4": {"value": -1, "coordinate": [3.50, 0.75, 0.85, 3.14159]},
    "D5": {"value": -1, "coordinate": [3.50, 1.25, 0.85, 3.14159]},
    "D6": {"value": -1, "coordinate": [3.50, 1.75, 0.85, 3.14159]},
}

# 查询value
def query_value(address):
    key = json_data.get(address)
    return key["value"] if key is not None else None


# 查询coordinate
def query_coordinate(address):
    key = json_data.get(address)
    return key["coordinate"] if key is not None else None


# 更新value
def update_json_value(address, new_value):
    if address in json_data:
        json_data[address]["value"] = new_value
        # return json_data
    else:
        # 如果键不存在，可以选择抛出异常或者返回原始数据
        raise ValueError(f"Key '{address}' not found in JSON data.")
        # return json_data


# 评委根据值查询地址
def get_address_by_value(value):
    address = [key for key, val in json_data.items() if val["value"] == value]
    return address[0]


# 所有货物信息发送到串口屏
def search_all():
    i = 0
    for address, value in json_data.items():
        ser.write(f"page4.n{i}.val={value['value']}".encode("utf-8") + b"\xff\xff\xff")
        i += 1


# 货物编号为1～24的数值；坐标信息为A1~A6、B1~B6、C1~C6、D1~D6
def goods_callback(msg):
    if msg.address == 'kun':
        ser.write(f'page2.n0.val={msg.value}'.encode("utf-8") + b"\xff\xff\xff")
        # 路径发布
        coordinate=coordinate_info()
        coordinate.x=query_coordinate(get_address_by_value(msg.value))[0]
        coordinate.y=query_coordinate(get_address_by_value(msg.value))[1]
        coordinate.z=query_coordinate(get_address_by_value(msg.value))[2]
        coordinate.yaw=query_coordinate(get_address_by_value(msg.value))[3]
        path_pub.publish(coordinate)
        return
    update_json_value(msg.address, msg.value)
    ser.write(f'page1.t4.txt="{msg.value}"'.encode("utf-8") + b"\xff\xff\xff")
    ser.write(f'page1.t6.txt="{msg.address}"'.encode("utf-8") + b"\xff\xff\xff")  # 实时发送编号及坐标至串口屏


# 主程序
rospy.init_node("screen", anonymous=True)
# 配置串口
ser = serial.Serial("/dev/ttyS6", baudrate=9600, timeout=1)
# 起飞发布，发1为盘点程序，发2为定向程序
pub = rospy.Publisher("/offboard_order", Int32, queue_size=10)
# 从scanner订阅货物信息
rospy.Subscriber("/goods_info", goods_info, goods_callback)
# 路径发布者
path_pub = rospy.Publisher("/coordinate_info", coordinate_info, queue_size=10)

ser.write(b"rest\xff\xff\xff")  # 重置屏幕

while not rospy.is_shutdown():
    if ser.in_waiting > 0:
        try:
            line = ser.readline().decode("utf-8").strip()  # 读取串口数据
            rospy.loginfo(line)
            if line == "offboard1":  # 1盘点
                rospy.loginfo("offboard cmd1 from serial")
                pub.publish(Int32(1))
            elif line == "offboard2":  # 2定向
                rospy.loginfo("offboard cmd2 from serial")
                pub.publish(Int32(2))
            elif line == "search_all":  # 串口屏显示所有货物信息
                search_all()
            elif line.startswith("search"):  # 评委查询编号
                num = int(line[6:])
                ser.write(
                    f'page3.t4.txt="{get_address_by_value(num)}"'.encode("utf-8")
                    + b"\xff\xff\xff"
                )  # 评委查询编号后发送坐标至串口屏
        except Exception as e:
            rospy.loginfo("exc")
            pass
