/**
 * @file actuation_node.cpp
 * @author Haowen Shi
 * @date 16 Mar 2019
 * @brief Locomotion and actuator control node
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <pthread.h>
#include "mecanum_ctrl.hpp"

#define MSG_QUEUE_SIZE (1)

int16_t motor_vels[4];
ros::Publisher motor_vel_pub;

void print_usage(void) {
  printf("usage: rosrun jr_actuation actuation\n");
}

void lwheel_vtarget_callback(geometry_msgs::Twist vtarget) {
    mecanum_calc(
      vtarget.linear.x, vtarget.linear.y, vtarget.angular.z, motor_vels);

    // Publish command velocities for motors
    geometry_msgs::Quaternion motor_vel_msg;
    motor_vel_msg.x = motor_vels[MOTOR_FR];
    motor_vel_msg.y = motor_vels[MOTOR_FL];
    motor_vel_msg.z = motor_vels[MOTOR_BL];
    motor_vel_msg.w = motor_vels[MOTOR_BR];
    motor_vel_pub.publish(motor_vel_msg);
}

int main(int argc, char **argv) {
    int err;
    pthread_t comm_thread;

    if (argc < 1) {
        print_usage();
        return -1;
    }

    // More initialization here

    ros::init(argc, argv, "jr_actuation");

    ros::NodeHandle node;

    motor_vel_pub = node.advertise<geometry_msgs::Quaternion>(
      "jr_motor_vel", MSG_QUEUE_SIZE);

    /*
     * Topic : jr_cmd_vel
     * Type  : geometry_msgs/Twist
     */
    ros::Subscriber sub = node.subscribe(
      "jr_cmd_vel", MSG_QUEUE_SIZE, lwheel_vtarget_callback);

    ROS_INFO("initialized actuation node");

    ros::spin();

    return 0;
}