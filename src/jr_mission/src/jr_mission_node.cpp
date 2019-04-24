/**
 * @file mission_node.cpp
 * @author Haowen Shi
 * @date 13 Apr 2019
 * @brief This is Houston.
 */

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <pthread.h>
#include "parser.hpp"

#define MSG_QUEUE_SIZE (1)

int16_t motor_vels[4];
ros::Publisher cmd_vel_pub;

/**
 * Current chassis pose, updated by localization node
 */
geometry_msgs::Pose chassisPose;

void print_usage(void) {
    printf("usage: TODO\n");
}

void localization_callback(geometry_msgs::Pose pose) {
    chassisPose = pose;
}

int main(int argc, char **argv) {
    int err;

    if (argc < 1) {
        print_usage();
        return -1;
    }

    // More initialization here

    // Parse mission file and queue tasks up

    ros::init(argc, argv, "jr_mission");

    ros::NodeHandle node;

    cmd_vel_pub = node.advertise<geometry_msgs::Twist>(
      "jr_cmd_vel", MSG_QUEUE_SIZE);

    /*
     * Topic : jr_chassis_pose
     * Type  : geometry_msgs/Pose
     */
    ros::Subscriber sub = node.subscribe(
      "jr_chassis_pose", MSG_QUEUE_SIZE, localization_callback);

    ROS_INFO("initialized actuation node");

    ros::spin();

    return 0;
}