#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <ros_tools/LidarPose.h>
#include <ros_tools/message_subscriber.h>
#include <ros_tools/target_class.hpp>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <string>
#include <trx_screen/coordinate_info.h>
#include <trx_screen/goods_info.h>
#include <vector>

bool need_scan(int target_index) {
    return target_index >= 1 && target_index <= 6 ||
           target_index >= 9 && target_index <= 14 ||
           target_index >= 16 && target_index <= 21 ||
           target_index >= 24 && target_index <= 29;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "offboard");
    ros::NodeHandle nh;

    mavros_msgs::State current_state;
    ros_tools::LidarPose lidar_pose_data;
    std_msgs::Int32 barcode_data, barcode_data_vision, offboard_order;
    std_msgs::Float32 vert_deviation;

    bool scanned = false;

    auto barcode_cb = [&barcode_data, &scanned](const auto &msg) {
        barcode_data = *msg;
        if (barcode_data.data != -1) {
            scanned = true;
        }
    };

    std::vector<target> targets2;

    auto coordinate_cb = [&targets2](const auto &msg) {
        trx_screen::coordinate_info coordinate_info = *msg;
        targets2 = {target(0, 0, 1.28, 0),
                    target(0, -0.25, 1.28, coordinate_info.yaw),
                    target(coordinate_info.x, -0.25, coordinate_info.z,
                           coordinate_info.yaw),
                    target(coordinate_info.x, coordinate_info.y,
                           coordinate_info.z, coordinate_info.yaw),
                    target(coordinate_info.x, 2.75, coordinate_info.z,
                           coordinate_info.yaw),
                    target(3.5, 2.75, coordinate_info.z, coordinate_info.yaw),
                    target(3.5, 2.5, coordinate_info.z, 0),
                    target(3.5, 2.5, 0.1, 0)};
    };

    auto state_sub = subscribe(nh, "/mavros/state", current_state),
         lidar_sub = subscribe(nh, "/lidar_data", lidar_pose_data),
         barcode_sub =
             nh.subscribe<std_msgs::Int32>("/barcode_data", 10, barcode_cb),
         barcode_vision_sub =
             subscribe(nh, "/barcode_data_vision", barcode_data_vision),
         offboard_sub = subscribe(nh, "/offboard_order", offboard_order),
         coordinate_sub = nh.subscribe<trx_screen::coordinate_info>(
             "/coordinate_info", 10, coordinate_cb),
         vert_deviation_sub = subscribe(nh, "/vert_deviation", vert_deviation);

    auto local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
             "/mavros/setpoint_position/local", 10),
         goods_info_pub =
             nh.advertise<trx_screen::goods_info>("/goods_info", 10),
         laser_and_led_pub =
             nh.advertise<std_msgs::Int32>("/laser_and_led_order", 10);

    auto arming_client =
             nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming"),
         command_client =
             nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command"),
         set_mode_client =
             nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    ros::Rate rate(20.0);

    std::vector<target> targets = {
        target(0.0, 0.0, 1.35, 0), // 起飞高度1.5,面西

        target(0.0, 0.75, 1.28, 0), // A3,上排
        target(0.0, 1.25, 1.28, 0), // A2
        target(0.0, 1.75, 1.28, 0), // A1
        target(0.0, 1.75, 0.88, 0), // A4,下排
        target(0.0, 1.25, 0.88, 0), // A5
        target(0.0, 0.75, 0.88, 0), // A6

        target(0.00, -0.25, 0.88, 0), // 过面
        target(1.75, -0.25, 0.88, 0),

        target(1.75, 0.75, 0.88, 0), // C6,下排
        target(1.75, 1.25, 0.88, 0), // C5
        target(1.75, 1.75, 0.88, 0), // C4
        target(1.75, 1.75, 1.28, 0), // C1,上排
        target(1.75, 1.25, 1.28, 0), // C2
        target(1.75, 0.75, 1.28, 0), // C3

        target(1.75, 0.75, 1.28, M_PI), // 转向

        target(1.75, 0.75, 1.28, M_PI), // B1,上排
        target(1.75, 1.25, 1.28, M_PI), // B2
        target(1.75, 1.75, 1.28, M_PI), // B3
        target(1.75, 1.75, 0.88, M_PI), // B6,下排
        target(1.75, 1.25, 0.88, M_PI), // B5
        target(1.75, 0.75, 0.88, M_PI), // B4

        target(1.75, -0.25, 0.88, M_PI), // 过面
        target(3.50, -0.25, 0.88, M_PI),

        target(3.50, 0.75, 0.88, M_PI), // D4,下排
        target(3.50, 1.25, 0.88, M_PI), // D5
        target(3.50, 1.75, 0.88, M_PI), // D6
        target(3.50, 1.75, 1.28, M_PI), // D1,上排
        target(3.50, 1.25, 1.28, M_PI), // D2
        target(3.50, 0.75, 1.28, M_PI), // D3

        target(3.50, 2.5, 1.28, M_PI),   target(3.50, 2.5, 0.10, M_PI) // 降落
    };

    std::vector<std::string> addresses = {
        "",   "A3", "A2", "A1", "A4", "A5", "A6", "",   "",   "C6",
        "C5", "C4", "C1", "C2", "C3", "",   "B1", "B2", "B3", "B6",
        "B5", "B4", "",   "",   "D4", "D5", "D6", "D3", "D2", "D1"};

    bool has_scanned[25]{};

    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    std_msgs::Int32 laser_and_led_order;
    laser_and_led_order.data = 1;

    size_t target_index = 0, target_index2 = 0;
    int mode = 0;

wait_for_command:
    offboard_order.data = 0;

    while (ros::ok() && !offboard_order.data) {
        ros::spinOnce();
        rate.sleep();
    }

    scanned = false;

    if (offboard_order.data == 2) {
        mode = 2;
        while (!scanned) {
            ros::spinOnce();
            rate.sleep();
        }
        // 识别到二维码
        ROS_INFO("Barcode detected: %d", barcode_data.data);
        trx_screen::goods_info goods_info;
        goods_info.value = barcode_data.data;
        goods_info.address = "kun";
        goods_info_pub.publish(goods_info); // 不完全发送，仅value可用
        scanned = false;
        while (!targets2.size()) {
            ros::spinOnce();
            rate.sleep();
        }
        offboard_order.data = 0;
        while (ros::ok() && !offboard_order.data) {
            ros::spinOnce();
            rate.sleep();
        }
    }

    ros::Time last_request = ros::Time::now();

    while (ros::ok()) {
        if (!current_state.armed &&
            (ros::Time::now() - last_request > ros::Duration(1.0))) {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        } else if (current_state.mode != "OFFBOARD" &&
                   (ros::Time::now() - last_request > ros::Duration(1.0))) {
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = 0;
                pose.pose.position.y = 0;
                pose.pose.position.z = 0.5;
                local_pos_pub.publish(pose);
            }
            last_request = ros::Time::now();
        }
        if (current_state.armed && current_state.mode == "OFFBOARD") {
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok()) {
        if (mode == 0) { // 正常巡线
            if (target_index >= targets.size()) {
                ROS_INFO("All targets reached");
                mavros_msgs::CommandLong command_srv;
                command_srv.request.broadcast = false;
                command_srv.request.command = 21;
                command_srv.request.confirmation = 0;
                command_srv.request.param4 = 0;
                if (command_client.call(command_srv) &&
                    command_srv.response.success) {
                    ROS_INFO("Land command sent successfully");
                }
                break;
            } else if (!targets[target_index].pos_check(lidar_pose_data, 0.05,
                                                        0.05, 0.02)) {
                targets[target_index].fly_to_target(local_pos_pub);
            } else {
                ROS_INFO("Reached target %zu", target_index);
                if (need_scan(target_index)) {
                    mode = 1;
                    last_request = ros::Time::now();
                } else
                    target_index++;
            }
        } else if (mode == 1) { // 二维码扫描
            if (scanned) {
                if (!has_scanned[barcode_data.data]) {
                    // 识别到二维码，触发动作
                    ROS_INFO("Barcode detected: %d", barcode_data.data);
                    trx_screen::goods_info goods_info;
                    goods_info.value = barcode_data.data;
                    goods_info.address = addresses[target_index];
                    goods_info_pub.publish(goods_info);
                    laser_and_led_pub.publish(laser_and_led_order);
                    ros::Duration(0.5).sleep();
                    target_index++;
                    mode = 0;
                    scanned = false;
                    if (barcode_data.data > 0)
                        has_scanned[barcode_data.data] = true;
                }
            } else {
                // 未识别到二维码，巡线
                targets[target_index].z += vert_deviation.data; // 视觉修正
                targets[target_index].fly_to_target(local_pos_pub);
            }
            if (ros::Time::now() - last_request > ros::Duration(3.0)) {
                // 改用视觉识别
                ROS_INFO("Barcode detected by vision: %d",
                         barcode_data_vision.data);
                trx_screen::goods_info goods_info;
                goods_info.value = barcode_data_vision.data;
                goods_info.address = addresses[target_index];
                goods_info_pub.publish(goods_info);
                laser_and_led_pub.publish(laser_and_led_order);
                ros::Duration(0.5).sleep();
                target_index++;
                mode = 0;
                scanned = false;
                if (barcode_data_vision.data > 0)
                    has_scanned[barcode_data_vision.data] = true;
            }
        } else if (mode == 2) { // 发挥部分
            if (target_index2 >= targets2.size()) {
                ROS_INFO("All targets2 reached");
                mavros_msgs::CommandLong command_srv;
                command_srv.request.broadcast = false;
                command_srv.request.command = 21;
                command_srv.request.confirmation = 0;
                command_srv.request.param4 = 0;
                if (command_client.call(command_srv) &&
                    command_srv.response.success) {
                    ROS_INFO("Land command sent successfully");
                }
                break;
            } else if (!targets2[target_index2].pos_check(lidar_pose_data, 0.05,
                                                          0.05, 0.02)) {
                targets2[target_index2].fly_to_target(local_pos_pub);
            } else {
                ROS_INFO("Reached target %zu", target_index2);
                if (target_index2 == 3) {
                    last_request = ros::Time::now();
                    while (abs(vert_deviation.data) > 0.1) {
                        targets2[target_index2].z += vert_deviation.data;
                        targets2[target_index2].fly_to_target(local_pos_pub);
                        if (ros::Time::now() - last_request > ros::Duration(3.0))
                            break;
                        ros::spinOnce();
                        rate.sleep();
                    }
                    trx_screen::goods_info goods_info;
                    goods_info.address = "heizi";
                    goods_info_pub.publish(goods_info); // 货物信息回传
                    laser_and_led_pub.publish(laser_and_led_order);
                    ros::Duration(0.5).sleep();
                }
                target_index2++;
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

    if (offboard_order.data == 1)
        goto wait_for_command;

    return 0;
}