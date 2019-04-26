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
#include <string>
#include <signal.h>
#include <pthread.h>
#include <std_msgs/String.h>
#include <jr_communication/ChassisInfo.h>
#include <jr_communication/MotorCmd.h>

using namespace jr_communication;

#define MSG_QUEUE_SIZE (1)
#define UART_BUFF_SIZE (1000)
#define COMPUTER_FRAME_BUFLEN UART_BUFF_SIZE

/* Chassis information */
#define PID_VEL_P (0.1f)
#define PID_VEL_I (0.04f)
#define PID_VEL_D (0.14f)
#define PID_POS_P (0.1f)
#define PID_POS_I (0.0f)
#define PID_POS_D (0.0f)
#define CHASSIS_WHEEL_PERIMETER (314) // mm
#define CHASSIS_WHEEL_TRACK (500) // mm
#define CHASSIS_WHEEL_BASE  (550) // mm

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
  printf("usage: rosrun jr_comm_node tty_path <spd_p> <spd_i> <spd_d>\n");
}

bool send_command_callback(MotorCmd::Request  &request,
                           MotorCmd::Response &respond) {
  if (request.reset != 0) {
    jrcomm_send_chassis_reset();
    return true;
  }

  jrcomm_send_chassis_command(request.x_spd, request.y_spd, request.w_spd);
  return true;
}

void sigint_handler(int signal) {
  jrcomm_send_chassis_command(0, 0, 0);
  exit(0);
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

  //fprintf(stderr, "vel_p = %.5f", std::stof(argv[2], NULL));
  // Send initial chassis configuration
  infantry_structure_t config = {
    .chassis_config = CUSTOM_CONFIG,
    .wheel_perimeter = CHASSIS_WHEEL_PERIMETER,
    .wheel_track = CHASSIS_WHEEL_TRACK,
    .wheel_base = CHASSIS_WHEEL_BASE,
    .pid_vel_p = argc > 4 ? std::stof(argv[2], NULL) : PID_VEL_P,
    .pid_vel_i = argc > 4 ? std::stof(argv[3], NULL) : PID_VEL_I,
    .pid_vel_d = argc > 4 ? std::stof(argv[4], NULL) : PID_VEL_D,
    .pid_pos_p = PID_POS_P,
    .pid_pos_i = PID_POS_I,
    .pid_pos_d = PID_POS_D
  };
  jrcomm_send_chassis_config(config);
  printf("sent chassis config data\n");

  pthread_create(&comm_thread, NULL, jrcom_recv_thread, NULL);

  chassis_info_pub = node.advertise<jr_communication::ChassisInfo>(
          "jr_chassis", MSG_QUEUE_SIZE);

  comm_serv = node.advertiseService("jr_comm_cmd", send_command_callback);

  // Running at 5Hz
  ros::Rate loop_rate(5);

  // signal(SIGINT, sigint_handler);

  while(ros::ok()) {
    chassis_info_pub.publish(chassis_info_msg);

    // Debug testing
    //jrcomm_send_chassis_command(1, 2, 3);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
