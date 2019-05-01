/**
 * @file jr_kinematics_node.cpp
 * @author Haowen Shi, Sara Misra
 * @date Apr 2019
 * @brief ROS interface for Hebi arm IK and trajectory execution
 */

#include <ros/ros.h>
#include <jr_kinematics/ExecuteArm.h>
#include <jr_kinematics/CorrectArm.h>
#include <jr_kinematics/ExecuteHand.h>
#include <pthread.h>
#include <signal.h>
#include <atomic>
#include <kinematics.h>

using namespace jr_kinematics;

/** @brief Protects execution, should use mutex, hack for now */
std::atomic_bool execution_in_progress = false;

void print_usage(void) {
  printf("usage: rosrun jr_kinematics\n");
}

/* Blocking operation for arm execution */
bool execute_arm_callback(ExecuteArm::Request  &request,
                        ExecuteArm::Response &respond) {
  if (execution_in_progress) {
    ROS_WARN("arm execution in progress");
    return false;
  }
  execution_in_progress = true;
  // TODO: add execute arm callback
  motion traj; 
  request.target_pose.position.z += request.z_offset;
  respond.done = traj.exec_traj(request.target_pose, request.device_orient, request.intial_rot, true);
  execution_in_progress = false;
  return true;
}

/* Blocking operation for arm z correction */
bool correct_arm_callback(CorrectArm::Request  &request,
                          CorrectArm::Response &respond) {
  if (execution_in_progress) {
    ROS_WARN("arm execution in progress");
    return false;
  }
  execution_in_progress = true;
  // TODO: add correct arm callback
  execution_in_progress = false;
  return true;
}

/* Blocking operation for arm z correction */
bool execute_hand_callback(ExecuteHand::Request  &request,
                           ExecuteHand::Response &respond) {
  if (execution_in_progress) {
    ROS_WARN("arm execution in progress");
    return false;
  }
  execution_in_progress = true;
  // TODO: add execute hand callback
  execution_in_progress = false;
  return true;
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
