/**
 * @file actuation_node.cpp
 * @author Haowen Shi
 * @date 16 Mar 2019
 * @brief Locomotion and actuator control node
 */

#include <ros/ros.h>
#include <chrono>
#include <thread>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <jr_communication/MotorCmd.h>
#include <jr_communication/ChassisInfo.h>
#include <jr_actuation/LocomotionCmd.h>
#include "jr_common.h"

using namespace jr_actuation;

#define MSG_QUEUE_SIZE (1)

#define TOF_TOUT_LIMIT (15)

#define POS_TOLERANCE (15)
#define MIN_DIST_TO_SWAP_RAIL (100)
#define DESIRED_TO_GUIDERAIL_DIST (80)
#define TRANSLATION_SPD (200) // unit TBD

enum GuideRail currentRail = DEFAULT_RAIL;

ros::ServiceClient vel_srv_client;

int tof_L_timeout = 0;
int tof_R_timeout = 0;
int tof_F_timeout = 0;

int tof_L_prev = -1;
int tof_R_prev = -1;
int tof_F_prev = -1;

/**
 * Station Positions in terms of TOF reading
 * First 5 are readings of R sensor (RAIL_LONG)
 * Last 3 are readings of L sensor (RAIL_SHORT)
 */
int stationPositions[NUM_STATIONS] =
  {1315, 1000, 558, 242, 10, 10, 260, 500};

int TOF_L_Reading; // left TOF reading
int TOF_R_Reading; // right TOF reading
int TOF_F_Reading; // front TOF reading
int SW_L_State;    // left limit switch reading
int SW_R_State;    // right limit switch reading

bool executionInProgress = 0;

bool resetRequired = false;

void print_usage(void) {
  printf("usage: rosrun jr_actuation jr_acturation_node\n");
}

void wait_ms(int ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void chassis_reset() {
  jr_communication::MotorCmd cmd;
  cmd.request.reset = 1;
  vel_srv_client.call(cmd);
  ROS_ERROR("--performed chassis reset");
  wait_ms(600);
  resetRequired = false;
}

void chassis_reset_if_required(void) {
  if (resetRequired) chassis_reset();
}

void chassis_move_vel(geometry_msgs::Twist t) {
  // chassis_reset_if_required();
  jr_communication::MotorCmd cmd;
  cmd.request.ctrl_mode = 4; // don't care
  cmd.request.x_spd = t.linear.x;
  cmd.request.y_spd = t.linear.y;
  cmd.request.w_spd = t.angular.z;
  if (vel_srv_client.call(cmd)) {
    wait_ms(10); // prevent fast loop
    return;
  } else {
    ROS_ERROR("chassis_move_vel failed");
  }
}

void chassis_stop(void) {
  geometry_msgs::Twist zeroTwist;
  zeroTwist.linear.x = 0;
  zeroTwist.linear.y = 0;
  zeroTwist.angular.z = 0;
  chassis_move_vel(zeroTwist);
}

/**
 * @brief Runs forward into wall until both limit switches are depressed,
 *        then move back a bit, making robot parallel to the wall
 */
void calibrate_against_rail() {
  // chassis_reset_if_required();
  geometry_msgs::Twist tgt_vel;
  tgt_vel.linear.y = tgt_vel.angular.z = 0;
  tgt_vel.linear.x = TRANSLATION_SPD + 100;
  
  while (true) {
    wait_ms(10);
    if (SW_L_State == 1 && SW_R_State == 1) break;
    else { chassis_move_vel(tgt_vel); }
  }
  chassis_stop();

  if (currentRail == RAIL_LONG) {
    tgt_vel.linear.x = - TRANSLATION_SPD;
    tgt_vel.linear.y = TRANSLATION_SPD;
  } else if (currentRail == RAIL_SHORT) {
    tgt_vel.linear.x = - TRANSLATION_SPD;
    tgt_vel.linear.y = - TRANSLATION_SPD;
  }

  // int error;
  // do
  // {
  //   error = TOF_F_Reading - DESIRED_TO_GUIDERAIL_DIST;
  //   tgt_vel.linear.x = error > 0 ? - TRANSLATION_SPD : TRANSLATION_SPD;
  //   tgt_vel.linear.y = error > 0 ? TRANSLATION_SPD : - TRANSLATION_SPD;
  //   chassis_move_vel(tgt_vel);
  // } while (abs(error) > POS_TOLERANCE);
  chassis_move_vel(tgt_vel);
  wait_ms(1200);
  chassis_stop();
}

void turn_right_90() {
  // chassis_reset_if_required();
  geometry_msgs::Twist tgt_vel;
  tgt_vel.linear.x = tgt_vel.linear.y = 0;

  tgt_vel.angular.z = - 300;

  chassis_move_vel(tgt_vel);
  wait_ms(2700);
  chassis_stop();
}

void turn_left_90() {
  // chassis_reset_if_required();
  geometry_msgs::Twist tgt_vel;
  tgt_vel.linear.x = tgt_vel.linear.y = tgt_vel.angular.z = 0;

  tgt_vel.angular.z = 300;

  chassis_move_vel(tgt_vel);
  wait_ms(2700);
  chassis_stop();
}

/**
 * @brief Runs towards left until R TOF is within good range
 */
void locomote_from_long_to_short() {
  // chassis_reset_if_required();
  assert(currentRail == RAIL_LONG);

  geometry_msgs::Twist tgt_vel;
  tgt_vel.linear.x = tgt_vel.linear.y = tgt_vel.angular.z = 0;
  
  if (TOF_R_Reading < MIN_DIST_TO_SWAP_RAIL) {
    tgt_vel.linear.y = TRANSLATION_SPD;
    chassis_move_vel(tgt_vel);
    sleep(3);
    chassis_stop();
    sleep(1);
  }

  tgt_vel.linear.x = - TRANSLATION_SPD;
  tgt_vel.linear.y = 0;
  chassis_move_vel(tgt_vel);
  wait_ms(1000);

  turn_right_90();
  
  calibrate_against_rail();
}

void locomote_from_short_to_long() {
  // chassis_reset_if_required();
  assert(currentRail == RAIL_SHORT);

  geometry_msgs::Twist tgt_vel;
  tgt_vel.linear.x = tgt_vel.linear.y = tgt_vel.angular.z = 0;
  
  if (TOF_L_Reading < MIN_DIST_TO_SWAP_RAIL) {
    tgt_vel.linear.y = - TRANSLATION_SPD;
    chassis_move_vel(tgt_vel);
    sleep(3);
    chassis_stop();
    sleep(1);
  }

  tgt_vel.linear.x = - TRANSLATION_SPD;
  tgt_vel.linear.y = 0;
  chassis_move_vel(tgt_vel);
  wait_ms(1000);

  turn_left_90();
  
  calibrate_against_rail();
}

void slide_to_station(enum StationID s) {
  // chassis_reset_if_required();
  enum GuideRail r = GET_STATION_RAIL(s);
  assert(r == currentRail);

  geometry_msgs::Twist tgt_vel;
  tgt_vel.linear.x = tgt_vel.linear.y = tgt_vel.angular.z = 0;
  

  switch (r) {
    int error;
    case RAIL_LONG:
      // read R TOF distance
      do
      {
        // chassis_reset_if_required();
        error = TOF_R_Reading - stationPositions[s];
        // go right if positive error
        tgt_vel.linear.y = error > 0 ? - TRANSLATION_SPD : TRANSLATION_SPD;
        chassis_move_vel(tgt_vel);
      } while (abs(error) > POS_TOLERANCE);
      chassis_stop();
      
      break;
    case RAIL_SHORT:
      // read L TOF distance
      do
      {
        // chassis_reset_if_required();
        error = stationPositions[s] - TOF_L_Reading;
        // go right if positive error
        tgt_vel.linear.y = error > 0 ? - TRANSLATION_SPD : TRANSLATION_SPD;
        chassis_move_vel(tgt_vel);
      } while (abs(error) > POS_TOLERANCE);
      chassis_stop();
      
      break;
    default:
      ROS_FATAL("wrong rail state");
      assert(false);
  }
}

/**
 * Callback for locomotion command (go to which station)
 */
bool locomotion_cmd_callback(LocomotionCmd::Request &request,
                             LocomotionCmd::Response &response) {
  if (executionInProgress) {
    ROS_WARN("skipping locomotion callback b/c exec in progress");
    return true;
  }
  executionInProgress = true;
  chassis_reset_if_required();
  
  if (!GET_STATION_RAIL(request.station) == currentRail) {
    // need to move to the other rail
    if (currentRail == RAIL_LONG) {
      locomote_from_long_to_short();
      currentRail = RAIL_SHORT;
    } else {
      locomote_from_short_to_long();
      currentRail = RAIL_LONG;
    }
  }

  slide_to_station((enum StationID) request.station);

  executionInProgress = false;

  return true;
}

/**
 * Chassis feedback subscriber callback
 */
void chassis_fdb_callback(jr_communication::ChassisInfo info) {
  TOF_L_Reading = info.dist5;// == 0 ? TOF_L_Reading : info.dist5; //5
  TOF_R_Reading = info.dist3;// == 0 ? TOF_R_Reading : info.dist3; //3
  TOF_F_Reading = info.dist4;// == 0 ? TOF_F_Reading : info.dist4; //4
  SW_L_State = info.sw_l;
  SW_R_State = info.sw_r;

  if (TOF_L_Reading == tof_L_prev) {
    tof_L_timeout ++;
  } else {
    tof_L_timeout = 0;
  }
  if (TOF_R_Reading == tof_R_prev) {
    tof_R_timeout ++;
  } else {
    tof_R_timeout = 0;
  }
  if (TOF_F_Reading == tof_F_prev) {
    tof_F_timeout ++;
  } else {
    tof_F_timeout = 0;
  }

  if (tof_L_timeout > TOF_TOUT_LIMIT ||
  tof_R_timeout > TOF_TOUT_LIMIT ||
  tof_F_timeout > TOF_TOUT_LIMIT) {
    if (!resetRequired) {
      ROS_ERROR("reset is required");
    }
    resetRequired = true;
  }
  
  tof_L_prev = TOF_L_Reading;
  tof_R_prev = TOF_R_Reading;
  tof_F_prev = TOF_F_Reading;

  //std::cout << "r_state " << SW_R_State << std::endl;
}

int main(int argc, char **argv) {
    int err;

    if (argc < 1) {
        print_usage();
        return -1;
    }

    ros::ServiceServer actuation_server;

    // More initialization here

    ros::init(argc, argv, "jr_actuation");

    ros::NodeHandle node;

    ros::Subscriber sub = node.subscribe(
      "jr_chassis", MSG_QUEUE_SIZE, chassis_fdb_callback
    );

    vel_srv_client = node.serviceClient<jr_communication::MotorCmd>(
      "jr_comm_cmd"
    );

    actuation_server = node.advertiseService(
      "jr_locomotion_cmd", locomotion_cmd_callback
    );

    //turn_right_90();
    // calibrate_against_rail();

    // motor_vel_pub = node.advertise<geometry_msgs::Quaternion>(
    //   "jr_cmd_vel", MSG_QUEUE_SIZE);

    /*
     * Topic : jr_cmd_vel
     * Type  : geometry_msgs/Twist
     */
    // ros::Subscriber sub = node.subscribe(
    //   "jr_cmd_vel", MSG_QUEUE_SIZE, lwheel_vtarget_callback);

    ROS_INFO("initialized actuation node");

    chassis_reset();

    ROS_INFO("reset chassis");

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    //ros::spin();

    return 0;
}