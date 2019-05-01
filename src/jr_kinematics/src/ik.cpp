/**
 * @file ik.cpp
 * @author Sara Misra, Haowen Shi
 * @date Apr 2019
 * @brief Core arm functionalities: IK, trajectory, execution
 */

#include <ros/ros.h>
#include <boost/date_time.hpp>
#include <cmath>
#include <chrono>
#include <thread>
#include "jr_common.h"
#include "kinematics.h"

using namespace std;
using namespace hebi;

#define NUM_JOINTS (4)

void check_jnts(Eigen::VectorXd jnts) {
	if ((jnts[0] <= -M_PI   || jnts[0] >= M_PI)   ||
		(jnts[1] <= -M_PI/2 || jnts[1] >= M_PI/2) ||
		(jnts[2] <= -M_PI   || jnts[2] >= M_PI)   ||
		(jnts[3] <= -M_PI   || jnts[3] >= M_PI)) 
	{
		ROS_FATAL("Joint Limits have been hit. Please put robot in correct configuration");
		exit(-1);
	}
}

motion::motion(std::string robot_desc_string) {
	/* Start the broadcasting to the arm */
	group = lookup.getGroupFromNames(
		{ "JollyRoger Arm" },
		{ "Base", "Elbow-1", "Elbow-2", "EndEffector" }
	);
	group_hand = lookup.getGroupFromNames(
		{ "JollyRoger Hand" },
		{ "Jammer" }
	);
	if (!group || !group_hand)
	{
		ROS_FATAL("Group not found!");
		exit(-1);
	}
	group->setCommandLifetimeMs(1000);
	cout<<group_hand->size()<<endl;
	hebi_feedback();

	//check_jnts(hebi_feedback());

	/* Setup the IK */
	// set joint upper and lower joint limits
	ll.resize(5); ul.resize(5);
	// ll(0)=-3.14 ; ul(0)=3.14;
	// ll(1)=-1.57 ; ul(1)=1.57;
	// ll(2)=-0.785398163 ; ul(2)=2.0;
	// ll(3)=-3.14 ; ul(3)=3.14;
	// ll(4)=-3.14 ; ul(4)=3.14;
	ll(0)=-1.57 ; ul(0)=1.57;
	ll(1)= -3.14 ; ul(1)=2.4;
	ll(2)=-3.14 ; ul(2)=3.14;
	ll(3)=-3.14 ; ul(3)=3.14;
	ll(4)=-3.14 ; ul(4)=3.14;

	if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
		ROS_ERROR("Failed to construct kdl tree");
	}
	bool got_chain = my_tree.getChain(chain_start,chain_end,chain);
	if(!got_chain){
		ROS_INFO("Unable to load KDL chain");
	}
	tracik_solver = new TRAC_IK::TRAC_IK(chain,ll,ul,timeout, eps,type=type);
	
	/* Seed for trac-ik*/
	joint_angle_track.resize(5);
	joint_angle_track[0] = 0.0590396 ;
	joint_angle_track[1] =  2.17705 ;
	joint_angle_track[2] = 2.71345 ;
	joint_angle_track[3] = 0.581935 ;
	joint_angle_track[4] = 0.0;

	/* Horizontal Waypoint for the trajectory */
	waypoint_h.resize(5);
	waypoint_h(0) = -0.704595; 
	waypoint_h(1) =    2.03822; 
	waypoint_h(2) =    2.37278; 
	waypoint_h(3) =    0.52097; 
	waypoint_h(4) = -0.0360714; 

	waypoint_v.resize(5);
	waypoint_v(0) = -1.378721; 
	waypoint_v(1) = 1.34551; 
	waypoint_v(2) = 0.609537; 
	waypoint_v(3) = -1.9422; 
	waypoint_v(4) = 0.203068; 

	/* Homing position in Joint Angles*/
	homing.resize(5);
	homing(0) = 0.0590396;
	homing(1) =  2.17705;
	homing(2) = 2.71345;
	homing(3) = 0.581935;
	homing(4) = 0.0;
	tf::Quaternion in_q = tf::createQuaternionFromRPY(0,M_PI/2,0); // x-vertical - point staight to the device
	orient_h.x = in_q.x();orient_h.y = in_q.y();
	orient_h.z = in_q.z();orient_h.w = in_q.w();

	tf::Quaternion in_q_v = tf::createQuaternionFromRPY(0,M_PI,M_PI/2); // x-vertical - point staight to the device
	orient_v.x = in_q_v.x();orient_v.y = in_q_v.y();
	orient_v.z = in_q_v.z();orient_v.w = in_q_v.w();


	KDL::Vector linear(0,0,0); 
	KDL::Vector angular(0.174533,0.174533,0);
	tolerances.vel = linear; tolerances.rot = angular;
}


KDL::Frame motion::getFK(KDL::JntArray joints){
	KDL::ChainFkSolverPos_recursive fk_solver(chain);
	std::cout<<joints(0)<<" " << joints(1)<<" " << joints(2)<<" " << joints(4) << joints(5)<<endl;
	KDL::Frame pose;
	fk_solver.JntToCart(joints,pose);
	double x,y,z,w;
	std::cout<<pose.p.x() <<  " " << pose.p.y() << " " << pose.p.z() <<endl;
	//cout<<target_pose.position.x << " "<< target_pose.position.y  <<" "<<target_pose.position.z <<endl;
	pose.M.GetQuaternion(x,y,z,w); 
	std::cout<< x << " " << y << " " <<z << " " <<w<<endl;
	return(pose);
}


KDL::JntArray motion::getIK(geometry_msgs::Pose target_pose){ 
	//valid = tracik_solver->getKDLChain(chain);
	KDL::Vector posit(target_pose.position.x, target_pose.position.y, target_pose.position.z);
	KDL::Rotation orient = KDL::Rotation::Quaternion(target_pose.orientation.x, target_pose.orientation.y,
													 target_pose.orientation.z, target_pose.orientation.w);

	KDL::Frame end_effector_pose(orient,posit);
	KDL::JntArray nominal(chain.getNrOfJoints());
  
	for (uint j=0; j<nominal.data.size(); j++ ) {
		nominal(j) = joint_angle_track[j];
	}

	int rc=tracik_solver->CartToJnt(nominal,end_effector_pose,result, tolerances);
	if (rc > 0) {

		ROS_INFO("ik succeed");
		
		for (uint j=0; j<result.data.size(); j++ ) {
		joint_angle_track[j] = result(j);
		std::cout << result(j) <<std::endl;
		}

		
	}
	else {
		ROS_INFO("ik failed");
	}
	std::cout << "rc:" << rc << std::endl;


	return(result);

}

/* MOTION: to_homing */
bool motion::to_homing(){ 
	ROS_INFO("Homing the robot"); 
	const double nan = std::numeric_limits<float>::quiet_NaN();
	Eigen::VectorXd current_position = hebi_feedback();
	int num_points=3;

	Eigen::MatrixXd positions(NUM_JOINTS, num_points);
	Eigen::MatrixXd velocities(NUM_JOINTS, num_points);
	Eigen::MatrixXd accelerations(NUM_JOINTS, num_points);

	positions << current_position[0], homing(0),homing(0),
	             current_position[1], homing(1),homing(1),
	             current_position[2], homing(2),homing(2),
	             current_position[3], homing(3),homing(3);

	
	velocities << 0, nan, 0,
                  0, nan, 0,
                  0, nan, 0,
                  0, nan, 0;

    accelerations << 0, nan, 0,
                     0, nan, 0,
                     0, nan, 0,
                     0, nan, 0;
	// The times to reach each waypoint (in seconds)
	Eigen::VectorXd time(num_points);
	time << 0, 8, 9;

	// Define trajectory
	auto trajectory = hebi::trajectory::Trajectory::createUnconstrainedQp(time, positions, &velocities, &accelerations);


	hebi::GroupCommand hand_cmd(group_hand->size());
	hebi::GroupCommand cmd(NUM_JOINTS);
	double period = 0.01;
	double duration = trajectory->getDuration();
	Eigen::VectorXd pos_cmd(NUM_JOINTS);
	Eigen::VectorXd vel_cmd(NUM_JOINTS);
	Eigen::VectorXd pos_cmd_hand(group_hand->size());
	pos_cmd_hand[0] = homing(4);
	reset_hold();

	for (double t = 0; t < duration; t += period)
	{
	  // Pass "nullptr" in to ignore a term.
	  trajectory->getState(t, &pos_cmd, &vel_cmd, nullptr);
	  //hand_traj->getState(t, &pos_cmd_hand, &vel_cmd_hand, nullptr);
	  cmd.setPosition(pos_cmd);
	  cmd.setVelocity(vel_cmd);
	  hand_cmd.setPosition(pos_cmd_hand);
	  group->sendCommand(cmd);
	  group_hand->sendCommand(hand_cmd);
	  std::this_thread::sleep_for(std::chrono::milliseconds((long int) (period * 1000)));
	}

	// after last iteration, the cmd should be hold target
	set_hold();

	/* Hold position in background */
	std::thread t([pos_cmd, pos_cmd_hand, period, this](){
		ROS_INFO("hold position thread");
		while (should_hold_pos()) {
			hebi::GroupCommand hold_cmd(NUM_JOINTS);
			hold_cmd.setPosition(pos_cmd);
			group->sendCommand(hold_cmd);

			hebi::GroupCommand hand_hold_cmd(1);
			hand_hold_cmd.setPosition(pos_cmd_hand);
			//group_hand->sendCommand(hand_hold_cmd);

			std::this_thread::sleep_for(
				std::chrono::milliseconds((long int) (period * 1000)));
		}
		ROS_INFO("hold position loop end");
	});

	t.detach();
	
	return true;
}

/* MOTION: exec_traj */
bool motion::exec_traj(geometry_msgs::Pose target_pose, int init_rot, int device_orient, bool hold){

	// Define nan variable for readability below
	ROS_INFO("Executing Traj\n");
	const double nan = std::numeric_limits<float>::quiet_NaN();
	int num_points=4;
	Eigen::VectorXd waypoint;
	double jammer_rot = init_rot*M_PI/180;
	switch(device_orient){
		case 0: {
			ROS_INFO("orient_v");
			target_pose.orientation = orient_v; 
			waypoint = waypoint_v; 
			break; 
		}
		case 1: {
			ROS_INFO("orient_h");
			target_pose.orientation = orient_h; 
			waypoint = waypoint_h; 
			break;
		}
	}
	KDL::JntArray target_position = getIK(target_pose);

	bool homed = to_homing();
	ROS_INFO("Finshed to_homing\n");
	Eigen::VectorXd current_position = hebi_feedback();
	
	Eigen::MatrixXd positions(NUM_JOINTS,num_points);
	Eigen::MatrixXd velocities(NUM_JOINTS,num_points);
	Eigen::MatrixXd accelerations(NUM_JOINTS,num_points);



	positions << current_position[0], homing(0), waypoint(0), target_position(0),        
             	 current_position[1], homing(1), waypoint(1), target_position(1),
             	 current_position[2], homing(2), waypoint(2), target_position(2),        
             	 current_position[3], homing(3), waypoint(3), target_position(3);

	velocities << 0, nan, nan, 0,
                  0, nan, nan, 0,
                  0, nan, nan, 0,
                  0, nan, nan, 0;

    accelerations << 0, nan, nan, 0,
                     0, nan, nan, 0,
                     0, nan, nan, 0,
                     0, nan, nan, 0;

	// The times to reach each waypoint (in seconds)
	Eigen::VectorXd time(num_points);
	// time << 0, 15, 25, 33;
	time << 0, 3, 17, 27;

	// Define trajectory
	auto trajectory = hebi::trajectory::Trajectory::createUnconstrainedQp(time, positions, &velocities, &accelerations);
	// Send trajectory
	hebi::GroupCommand cmd(NUM_JOINTS);
	hebi::GroupCommand hand_cmd(group_hand->size());
	double period = 0.01;
	double duration = trajectory->getDuration();
	Eigen::VectorXd pos_cmd(NUM_JOINTS);
	Eigen::VectorXd vel_cmd(NUM_JOINTS);

	// Send jammer command 
	Eigen::VectorXd pos_cmd_hand(group_hand->size());
	pos_cmd_hand[0] = jammer_rot;

	/* Break position holding before sending next command */
	reset_hold();

	/* Execution of trajectory is blocking */
	for (double t = 0; t < duration; t += period)
	{
		// Pass "nullptr" in to ignore a term.
		trajectory->getState(t, &pos_cmd, &vel_cmd, nullptr);
		cmd.setPosition(pos_cmd);
		cmd.setVelocity(vel_cmd);
		hand_cmd.setPosition(pos_cmd_hand);
		group->sendCommand(cmd);
		group_hand->sendCommand(hand_cmd);
		std::this_thread::sleep_for(
			std::chrono::milliseconds((long int) (period * 1000)));
	}

	// after last iteration, the cmd should be hold target
	if (hold) {
		set_hold();
	}

	/* Hold position in background */
	std::thread t([pos_cmd, pos_cmd_hand, period, this](){
		ROS_INFO("hold position thread");
		while (should_hold_pos()) {
			hebi::GroupCommand hold_cmd(NUM_JOINTS);
			hold_cmd.setPosition(pos_cmd);
			group->sendCommand(hold_cmd);
			std::this_thread::sleep_for(
				std::chrono::milliseconds((long int) (period * 1000)));
		}
		ROS_INFO("hold position loop end");
	});

	t.detach();
	
	return true;

}

/* MOTION: exec_hand */
bool motion::exec_hand(int rotate, int delta_z){
	Eigen::VectorXd current_pos = hebi_feedback();
	double jam_angle = rotate *M_PI/180;
	double period  = 0.01;
	Eigen::VectorXd pos(NUM_JOINTS);
	Eigen::VectorXd pos_cmd_hand(group_hand->size());
	if(delta_z == 0){ 
		pos_cmd_hand<<jam_angle;

	}
	else{ 
		target_pose.position.z+= delta_z;
		KDL::JntArray pos_ = getIK(target_pose);
		pos_cmd_hand << current_pos(4);
		pos << 	pos_(0), 
			pos_(1), 
			pos_(2), 
			pos_(3);
		
	}

	// Do not reset hold

	set_hold();
	hebi::GroupCommand hand_cmd(group_hand->size());
	hebi::GroupCommand cmd(NUM_JOINTS);
	if(delta_z==0){ 
		
		hand_cmd.setPosition(pos_cmd_hand); 
		group_hand->sendCommand(hand_cmd);
	}
	else {
		hand_cmd.setPosition(pos_cmd_hand); 
		cmd.setPosition(pos);
		group->sendCommand(cmd); 
		group_hand->sendCommand(hand_cmd);
	}

	return true;
}

/* MOTION: exec_correction */
bool motion::exec_correction(geometry_msgs::Pose corrected_pose){ 
	Eigen::VectorXd current_pos = hebi_feedback();
	double period  = 0.01;
	Eigen::VectorXd pos(NUM_JOINTS);
	Eigen::VectorXd hand_pos(group_hand->size());
	KDL::JntArray pos_ = getIK(corrected_pose);
	pos << 	pos_(0), 
			pos_(1), 
			pos_(2), 
			pos_(3);
	hand_pos<< current_pos(4);

	set_hold();
	hebi::GroupCommand hand_cmd(group_hand->size());
	hebi::GroupCommand cmd(NUM_JOINTS);
	hand_cmd.setPosition(hand_pos); 
	cmd.setPosition(pos);
	group->sendCommand(cmd); 
	group_hand->sendCommand(hand_cmd);

	/* Hold corrected position */
	std::thread t([pos, period, this]() {
		ROS_INFO("hold position thread");
		while (should_hold_pos()) {
			hebi::GroupCommand hold_cmd(NUM_JOINTS);
			hold_cmd.setPosition(pos);
			group->sendCommand(hold_cmd);
			std::this_thread::sleep_for(
				std::chrono::milliseconds((long int) (period * 1000)));
		}
		ROS_INFO("hold position loop end");
	});

	t.detach();
	return true;


}

void motion::reset_hold(void) {
	ROS_INFO("reset_hold()");
	should_hold_position = false;
}

void motion::set_hold(void) {
	ROS_INFO("set_hold()");
	should_hold_position = true;
}

bool motion::should_hold_pos(void) {
	return should_hold_position;
}

Eigen::VectorXd motion::hebi_feedback(){ 
	group->setFeedbackFrequencyHz(5);
	GroupFeedback group_fbk(group->size());
	if (group->getNextFeedback(group_fbk))
	{
		std::cout << "Got feedback. Positions are: " << std::endl << group_fbk.getPosition() << std::endl;
		return(group_fbk.getPosition());
	}
	group->clearFeedbackHandlers();

}
