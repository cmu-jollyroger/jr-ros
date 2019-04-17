/**
 * @file jr_comm_node.cpp
 * @author Haowen Shi, Fiona Li
 * @date 19 March 2019
 * @brief ROS interface for jr microcontroller serial communication
 */

#include "ros/ros.h"
#include "protocol.h"
#include "communicate.h"
#include "jrcomms.hpp"
#include <pthread.h>
#include <std_msgs/String.h>
#include <jr_communication/ChassisInfo.h>
#include <jr_communication/MotorCmd.h>

using namespace jr_communication;

#define MSG_QUEUE_SIZE (1)
#define UART_BUFF_SIZE (1000)
#define COMPUTER_FRAME_BUFLEN UART_BUFF_SIZE

int tty_fd;

/* UART receive buffer */
static uint8_t computer_rx_buf[UART_BUFF_SIZE];

/* UART transmit buffer */
static uint8_t computer_tx_buf[COMPUTER_FRAME_BUFLEN];

/* Global chassis info holder for recv */
jr_communication::ChassisInfo chassis_info_msg;


typedef enum
{
  INFO_TOF = 0,
  INFO_MOTOR  = 1,
} info_type;

void print_usage(void) {
  printf("usage: rosrun jr_comm_node\n");
}

bool send_command_callback(MotorCmd::Request  &request,
                           MotorCmd::Response &respond) {
  jrcomm_send_chassis_command(request.x_spd, request.y_spd, request.w_spd);
}

int main(int argc, char **argv) {
  if (argc < 1) {
    print_usage();
    return -1;
  }

  pthread_t comm_thread;

  ros::init(argc, argv, "jr_comm");

  ros::NodeHandle node;
  ros::Publisher chassis_info_pub;
  ros::ServiceServer comm_serv;

  printf("node initialized\n");

  // More initialization here
  if (jrcomm_init(argv[1]) < 0) {
    printf("[failed]: jrcomm_init()\n");
    return -2;
  }

  pthread_create(&comm_thread, NULL, jrcom_recv_thread, NULL);

  chassis_info_pub = node.advertise<jr_communication::ChassisInfo>(
          "jr_chassis", MSG_QUEUE_SIZE);

  comm_serv = node.advertiseService("jr_comm_cmd", send_command_callback);

  // Running at 10Hz
  ros::Rate loop_rate(10);

  while(ros::ok()) {
    chassis_info_pub.publish(chassis_info_msg);

    jrcomm_send_chassis_command(1, 2, 3);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}