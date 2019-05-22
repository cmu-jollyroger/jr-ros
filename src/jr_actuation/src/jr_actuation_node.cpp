/**
 * @file actuation_node.cpp
 * @author Haowen Shi
 * @date 16 Mar 2019
 * @brief Locomotion and actuator control node
 */

#include <ros/ros.h>
#include <ros/console.h>
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
#define DESIRED_TO_GUIDERAIL_DIST (100) // mm
#define TRANSLATION_SPD (200) // unit TBD

typedef enum
{
  SPD_CTRL       = 0,
  ENC_CTRL_X     = 1,
  ENC_CTRL_Y     = 2,
  ENC_CTRL_W     = 3
} move_cmd_e;

enum GuideRail currentRail = DEFAULT_RAIL;

ros::ServiceClient vel_srv_client;

/**
 * Station Positions in terms of TOF reading
 * First 5 are readings of R sensor (RAIL_LONG)
 * Last 3 are readings of L sensor (RAIL_SHORT)
 */
int stationPositions[NUM_STATIONS] =
  {
    1300,
    1000,
    620,
    250,
    10,

    10,
    90,
    380
  };

int TOF_L_Reading; // left TOF reading
int TOF_R_Reading; // right TOF reading
int TOF_F_Reading; // front TOF reading
int SW_L_State;    // left limit switch reading
int SW_R_State;    // right limit switch reading
int enc_exec_done; // whether encoder operation has finished

enum StationID currentStation = STATION_A;

bool executionInProgress = 0;

void print_usage(void) {
  printf("usage: rosrun jr_actuation jr_acturation_node\n");
}

void wait_ms(int ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

/* Chassis commands: non-blocking */

void chassis_reset() {
  jr_communication::MotorCmd cmd;
  cmd.request.reset = 1;
  vel_srv_client.call(cmd);
  ROS_INFO("--performed chassis reset");
  wait_ms(600);
}

void chassis_move_vel(geometry_msgs::Twist t) {
  jr_communication::MotorCmd cmd;
  cmd.request.ctrl_mode = 4; // don't care
  cmd.request.move_cmd = SPD_CTRL;
  cmd.request.x_spd = t.linear.x;
  cmd.request.y_spd = t.linear.y;
  cmd.request.w_spd = t.angular.z;
  cmd.request.reset = 0;
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

/* Encoder movement operations: blocking */

void chassis_enc_move_x(int16_t dist_mm) {
  jr_communication::MotorCmd cmd;
  cmd.request.ctrl_mode = 4; // don't care
  cmd.request.move_cmd = ENC_CTRL_X;
  cmd.request.x_spd = dist_mm;
  cmd.request.reset = 0;
  if (vel_srv_client.call(cmd)) {
    wait_ms(100);
    while (!enc_exec_done) continue;
    chassis_stop();
  } else {
    ROS_ERROR("chassis_enc_move_x failed");
  }
}

void chassis_enc_move_y(int16_t dist_mm) {
  jr_communication::MotorCmd cmd;
  cmd.request.ctrl_mode = 4; // don't care
  cmd.request.move_cmd = ENC_CTRL_Y;
  cmd.request.y_spd = dist_mm;
  cmd.request.reset = 0;
  if (vel_srv_client.call(cmd)) {
    wait_ms(100);
    while (!enc_exec_done) continue;
    chassis_stop();
  } else {
    ROS_ERROR("chassis_enc_move_y failed");
  }
}

void chassis_enc_turn_degrees(int16_t degrees) {
  /* Degrees in terms of vw */
  jr_communication::MotorCmd cmd;
  cmd.request.ctrl_mode = 4; // don't care
  cmd.request.move_cmd = ENC_CTRL_W;
  cmd.request.w_spd = degrees;
  cmd.request.reset = 0;
  if (vel_srv_client.call(cmd)) {
    wait_ms(100);
    while (!enc_exec_done) continue;
    chassis_stop();
  } else {
    ROS_ERROR("chassis_enc_turn_degrees failed");
  }
}

/* Primitive movement operations: blocking */

/**
 * @brief Runs forward into wall until both limit switches are depressed,
 *        then move back a bit, making robot parallel to the wall
 */
void calibrate_against_rail(int speed) {
  ROS_INFO("calibrate_against_rail()");
  geometry_msgs::Twist tgt_vel;
  tgt_vel.linear.y = tgt_vel.angular.z = 0;
  tgt_vel.linear.x = speed;
  
  /* Run forward until both limit switches are depressed */
  while (true) {
    wait_ms(10);
    if ((SW_L_State == 1 || currentStation == STATION_A) &&
        (SW_R_State == 1 || currentStation == STATION_H))
    {
      break;
    }
    else { chassis_move_vel(tgt_vel); }
  }
  chassis_stop();

  /* Move back, making robot parallel to the wall */
  //chassis_enc_move_x(-DESIRED_TO_GUIDERAIL_DIST);
  if (currentRail == RAIL_LONG) {
    tgt_vel.linear.x = - TRANSLATION_SPD;
    tgt_vel.linear.y = TRANSLATION_SPD;
  } else if (currentRail == RAIL_SHORT) {
    tgt_vel.linear.x = - TRANSLATION_SPD;
    tgt_vel.linear.y = - TRANSLATION_SPD;
  }
  chassis_move_vel(tgt_vel);
  wait_ms(600);
  chassis_stop();
}

void turn_right_90() {
  ROS_INFO("turn_right_90()");
  chassis_enc_turn_degrees(-90);
}

void turn_left_90() {
  ROS_INFO("turn_left_90()");
  chassis_enc_turn_degrees(90);
}

/**
 * @brief Runs towards left until R TOF is within good range
 */
void locomote_from_long_to_short() {
  ROS_INFO("locomote_from_long_to_short()");
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
  
  calibrate_against_rail(TRANSLATION_SPD + 100);
}

void locomote_from_short_to_long() {
  ROS_INFO("locomote_from_short_to_long()");
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
  
  calibrate_against_rail(TRANSLATION_SPD + 100);
}

void __slide_to_station(enum StationID s) {
  enum GuideRail r = GET_STATION_RAIL(s);
  assert(r == currentRail);

  ROS_INFO("__slide_to_station(%d)", (int)s);

  geometry_msgs::Twist tgt_vel;
  tgt_vel.linear.x = tgt_vel.linear.y = tgt_vel.angular.z = 0;
  
  switch (r) {
    int error; // in terms of mm
    case RAIL_LONG:
      // read R TOF distance
      error = TOF_R_Reading - stationPositions[s];
      // go right if positive error
      chassis_enc_move_y(- error);
      break;

    case RAIL_SHORT:
      // read L TOF distance
      error = stationPositions[s] - TOF_L_Reading;
      // go right if positive error
      chassis_enc_move_y(- error);
      break;

    default:
      ROS_FATAL("wrong rail state");
      chassis_stop();
      assert(false);
  }

  ROS_INFO("slide_to_station() complete");
}

void slide_to_station(enum StationID s) {
  __slide_to_station(s);
  calibrate_against_rail(200);
  wait_ms(3500);
  __slide_to_station(s);
}

/**
 * Callback for locomotion command (go to which station)
 */
bool locomotion_cmd_callback(LocomotionCmd::Request &request,
                             LocomotionCmd::Response &response) {
  if (executionInProgress) {
    ROS_WARN("skipping locomotion callback b/c exec in progress");
    response.status = -1;
    return false;
  }
  executionInProgress = true;
  
  currentStation = (enum StationID) request.station;

  if (!GET_STATION_RAIL(request.station) == currentRail) {
    // need to move to the other rail
    if (currentRail == RAIL_LONG) {
      ROS_INFO("locomote_from_long_to_short");
      locomote_from_long_to_short();
      currentRail = RAIL_SHORT;
    } else {
      ROS_INFO("locomote_from_short_to_long");
      locomote_from_short_to_long();
      currentRail = RAIL_LONG;
    }
  }

  ROS_INFO("slide_to_station");
  slide_to_station((enum StationID) request.station);

  executionInProgress = false;

  ROS_INFO("-- LOCOMOTION COMPLETE --");
  response.status = 0;
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
  enc_exec_done = info.enc_exec_done;

  //std::cout << "r_state " << SW_R_State << std::endl;
}

int main(int argc, char **argv) {
    int err;

    if (argc < 1) {
        print_usage();
        return -1;
    }

    ros::ServiceServer actuation_server;

    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
      ros::console::notifyLoggerLevelsChanged();
    }

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

    ROS_INFO("initialized actuation node");

    //chassis_reset();

    //ROS_INFO("reset chassis");

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    //ros::spin();

    return 0;
}
