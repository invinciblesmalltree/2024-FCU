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
#include <std_msgs/Int32.h>
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
    std_msgs::Int32 barcode_data, offboard_order;

    bool scanned = false;

    auto barcode_cb = [&barcode_data,
                       &scanned](const std_msgs::Int32::ConstPtr &msg) {
        barcode_data = *msg;
        if (barcode_data.data != -1)
            scanned = true;
    };

    auto state_sub = subscribe(nh, "/mavros/state", current_state),
         lidar_sub = subscribe(nh, "/lidar_data", lidar_pose_data),
         barcode_sub =
             nh.subscribe<std_msgs::Int32>("/barcode_data", 10, barcode_cb),
         offboard_sub = subscribe(nh, "/offboard_order", offboard_order);

    auto local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
             "/mavros/setpoint_position/local", 10),
         velocity_pub = nh.advertise<geometry_msgs::TwistStamped>(
             "/mavros/setpoint_velocity/cmd_vel", 10),
         led_pub = nh.advertise<std_msgs::Int32>("/led", 10),
         servo_pub = nh.advertise<std_msgs::Int32>("/servo", 10),
         screen_data_pub = nh.advertise<std_msgs::Int32>("/screen_data", 10);

    auto arming_client =
             nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming"),
         command_client =
             nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command"),
         set_mode_client =
             nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    ros::Rate rate(20.0);

    std::vector<target> targets = {
        target(0.0, 0.0, 1.35, 0), // 起飞高度1.5,面西

        target(0.0, 0.75, 1.25, 0), // A3,上排
        target(0.0, 1.25, 1.25, 0), // A2
        target(0.0, 1.75, 1.25, 0), // A1
        target(0.0, 1.75, 0.85, 0), // A4,下排
        target(0.0, 1.25, 0.85, 0), // A5
        target(0.0, 0.75, 0.85, 0), // A6

        target(0.00, -0.25, 0.85, 0), // 过面
        target(1.75, -0.25, 0.85, 0),

        target(1.75, 0.75, 0.85, 0), // C6,下排
        target(1.75, 1.25, 0.85, 0), // C5
        target(1.75, 1.75, 0.85, 0), // C4
        target(1.75, 1.75, 1.25, 0), // C1,上排
        target(1.75, 1.25, 1.25, 0), // C2
        target(1.75, 0.75, 1.25, 0), // C3

        target(1.75, 0.75, 1.25, M_PI), // 转向

        target(1.75, 0.75, 1.25, M_PI), // B3,上排
        target(1.75, 1.25, 1.25, M_PI), // B2
        target(1.75, 1.75, 1.25, M_PI), // B1
        target(1.75, 1.75, 0.85, M_PI), // B4,下排
        target(1.75, 1.25, 0.85, M_PI), // B5
        target(1.75, 0.75, 0.85, M_PI), // B6

        target(1.75, -0.25, 0.85, M_PI), // 过面
        target(3.50, -0.25, 0.85, M_PI),

        target(3.50, 0.75, 0.85, M_PI), // D6,下排
        target(3.50, 1.25, 0.85, M_PI), // D5
        target(3.50, 1.75, 0.85, M_PI), // D4
        target(3.50, 1.75, 1.25, M_PI), // D1,上排
        target(3.50, 1.25, 1.25, M_PI), // D2
        target(3.50, 0.75, 1.25, M_PI), // D3

        target(3.50, 2.5, 1.25, M_PI),   target(3.50, 2.5, 0.10, M_PI) // 降落
    };

    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    size_t target_index = 0;
    int mode = 0;

    while (ros::ok() && !offboard_order.data) {
        ros::spinOnce();
        rate.sleep();
    }

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
                }
                target_index++;
            }
        } else if (mode == 1) { // 二维码扫描
            if (scanned) {
                // 识别到二维码，触发动作
                ROS_INFO("Barcode detected: %d", barcode_data.data);
                target_index++;
                mode = 0;
                scanned = false;
            } else {
                // 未识别到二维码，巡线
                targets[target_index].fly_to_target(local_pos_pub);
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}