//
// Created by FionaLee on 3/19/19.
//

#include "jr_communication_node.h"
#include <ros/ros.h>

bool send_command_callback(MotorCmd::Request  &request,
                           MotorCmd::Response &respond) {
  comm_send_chassis_command(request.x_spd, request.y_spd, request.w_spd);
}


void print_usage(void) {
  printf("usage: rosrun jr_comm_cmd_node\n");
}

int main(int argc, char **argv) {
  if (argc < 1) {
    print_usage();
    return -1;
  }

  jrcomm_init("tty_name");
  ros::init(argc, argv, "jr_comm_cmd_node");
  ros::NodeHandle node;

  ros::ServiceServer service = node.advertiseService("jr_communication_cmd", send_command_callback);
  ros::spin();
  return 0;
}