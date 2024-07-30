# !/usr/bin/env python3
import rospy
import serial
from std_msgs.msg import Int32
from screen.msg import goods_info
import json

# 读取json文件
with open('goods_info.json', 'r') as json_file:
    json_data = json.load(json_file)

# 查询value
def query_value(element_name):
    element = json_data.get(element_name)
    return element['value'] if element is not None else None

# 查询coordinate
def query_coordinate(element_name):
    element = json_data.get(element_name)
    return element['coordinate'] if element is not None else None

# 更新value
def update_json_value(json_data, address, new_value):
    if address in json_data:
        json_data[address] = new_value
        return json_data
    else:
        # 如果键不存在，可以选择抛出异常或者返回原始数据
        raise ValueError(f"Key '{address}' not found in JSON data.")
        # return json_data

# 评委根据值查询地址
def get_address_by_value(json_data, value):
    address = [key for key, val in json_data.items() if val == value]
    return address

# 所有货物信息发送到串口屏
def search_all():
    i=0
    for address, value in json_data.items():
        ser.write(f"page4.n{i}.val={value}\xff\xff\xff".encode())
        i=i+1

# 货物编号为1～24的数值；坐标信息为A1~A6、B1~B6、C1~C6、D1~D6
def goods_callback(msg):
    update_json_value(json_data, msg.address, msg.value)
    ser.write(b"page1.t4.txt=%d\xff\xff\xff",msg.value)
    ser.write(b"page1.t6.txt=%d\xff\xff\xff",msg.address) # 实时发送编号及坐标至串口屏

# 配置串口
ser = serial.Serial("/dev/AMA0", baudrate=9600, timeout=1)
# 起飞发布，发1为盘点程序，发2为定向程序
pub = rospy.Publisher("/offboard_order", Int32, queue_size=10)
# 从scanner订阅货物信息
rospy.Subscriber("/barcode_data", goods_info, goods_callback)

# 主程序
rospy.init_node("screen", anonymous=True)
ser.write(b"rest\xff\xff\xff") # 重置屏幕

while not rospy.is_shutdown():
    if ser.in_waiting > 0:
        try:
            line = ser.readline().decode("utf-8").strip() # 读取串口数据
            if line == "offboard1": # 1盘点
                rospy.loginfo("offboard cmd1 from serial")
                pub.publish(Int32(1))
            elif line == "offboard2": # 2定向
                rospy.loginfo("offboard cmd2 from serial")
                pub.publish(Int32(2))
            elif line.startwith("search"): # 评委查询编号
                num=int(line[2:])
                ser.write(b"page3.t4.txt=\"%s\"\xff\xff\xff",get_address_by_value(json_file_path,num)) # 评委查询编号后发送坐标至串口屏
            elif line == "search_all": # 串口屏显示所有货物信息
                search_all()
        except Exception as e:
            pass

    rospy.sleep(0.1)