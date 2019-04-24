/**
 * @file actuation_node.cpp
 * @author Haowen Shi
 * @date 16 Mar 2019
 * @brief Locomotion and actuator control node
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <jr_communication/MotorCmd.h>
#include <jr_actuation/LocomotionCmd.h>
#include "jr_common.h"

using namespace jr_actuation;

#define MSG_QUEUE_SIZE (1)

enum GuideRail currentRail = DEFAULT_RAIL;

void print_usage(void) {
  printf("usage: rosrun jr_actuation jr_acturation_node\n");
}

/**
 * @brief Runs forward into wall until both limit switches are depressed,
 *        then move back a bit, making robot parallel to the wall
 */
void calibateAgainstWall() {

}

/**
 * @brief Runs towards right until right TOF is within good range
 */
void locomoteFromLongToShort() {

}

void locomoteFromShortToLong() {

}

/**
 * Callback for locomotion command (go to which station)
 */
bool locomotion_cmd_callback(LocomotionCmd::Request &request,
                             LocomotionCmd::Response *response) {
  if (GET_STATION_RAIL(request.station) == currentRail) {
    // no need to move to other rail
  } else {
    // need to move to the other rail
    if (currentRail == RAIL_LONG) { locomoteFromLongToShort(); }
    else { locomoteFromShortToLong(); }
  }
}

int main(int argc, char **argv) {
    int err;

    if (argc < 1) {
        print_usage();
        return -1;
    }

    // More initialization here

    ros::init(argc, argv, "jr_actuation");

    ros::NodeHandle node;

    // motor_vel_pub = node.advertise<geometry_msgs::Quaternion>(
    //   "jr_cmd_vel", MSG_QUEUE_SIZE);

    /*
     * Topic : jr_cmd_vel
     * Type  : geometry_msgs/Twist
     */
    // ros::Subscriber sub = node.subscribe(
    //   "jr_cmd_vel", MSG_QUEUE_SIZE, lwheel_vtarget_callback);

    ROS_INFO("initialized actuation node");

    ros::spin();

    return 0;
}