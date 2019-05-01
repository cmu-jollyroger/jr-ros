/**
 * @file jr_comm_node.cpp
 * @author Sara Misra, Haowen Shi
 * @date Apr 2019
 * @brief ROS interface for Hebi arm IK and trajectory execution
 */

#include <ros/ros.h>
#include <jr_kinematics/ExecuteArm.h>
#include <jr_kinematics/CorrectArm.h>
#include <jr_kinematics/ExecuteHand.h>
#include <pthread.h>
#include <signal.h>
#include <kinematics.h>
using namespace jr_kinematics;

void print_usage(void) {
  printf("usage: rosrun jr_kinematics\n");
}

/* Blocking operation for arm execution */
bool execute_arm_callback(ExecuteArm::Request  &request,
                        ExecuteArm::Response &respond) {
  // TODO: add execute arm callback

  
}

/* Blocking operation for arm z correction */
bool correct_arm_callback(CorrectArm::Request  &request,
                          CorrectArm::Response &respond) {
  // TODO: add correct arm callback
}

/* Blocking operation for arm z correction */
bool execute_hand_callback(ExecuteHand::Request  &request,
                           ExecuteHand::Response &respond) {
  // TODO: add execute hand callback
}

/**
 * @brief Loop responsible for running IK,
 *        sending arm commands and holding pos
 */
void *arm_command_loop(void *argu) {
  return NULL;
}

void sigint_handler(int signal) {
  /* Stop sending arm commands before shutdown */
  // TODO: add arm shutdown here
  ros::shutdown();
}

int main(int argc, char **argv) {
  if (argc < 1) {
    print_usage();
  }

  pthread_t arm_exec_thread;

  ros::init(argc, argv, "jr_kinematics");

  ros::NodeHandle node;
  ros::ServiceServer execute_arm_serv;
  ros::ServiceServer correct_arm_serv;
  ros::ServiceServer execute_hand_serv;

  execute_arm_serv = node.advertiseService(
    "jr_arm_execute_cmd", execute_arm_callback
  );
  correct_arm_serv = node.advertiseService(
    "jr_arm_correct_cmd", correct_arm_callback
  );
  execute_hand_serv = node.advertiseService(
    "jr_hand_execute_cmd", execute_hand_callback
  );

  /* Start arm command thread */
  pthread_create(&arm_exec_thread, NULL, arm_command_loop, NULL);

  // More initialization here

  ros::Rate loop_rate(5);

  signal(SIGINT, sigint_handler);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
