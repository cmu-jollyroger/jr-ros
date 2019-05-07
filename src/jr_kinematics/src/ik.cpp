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

bool motion::load_groups(){
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

}
motion::motion(std::string robot_desc_string) {
	
	load_groups();
	hebi_feedback_arm();

	//check_jnts(hebi_feedback());

	/* Setup the IK */
	// set joint upper and lower joint limits
	ll.resize(5); ul.resize(5);
	// ll(0)=-3.14 ; ul(0)=3.14;
	// ll(1)=-1.57 ; ul(1)=1.57;
	// ll(2)=-0.785398163 ; ul(2)=2.0;
	// ll(3)=-3.14 ; ul(3)=3.14;
	// ll(4)=-3.14 ; ul(4)=3.14;
	ll(0)=-3.14 ; ul(0)=3.14;
	ll(1)= 0.35 ; ul(1)=3.14;
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
	// joint_angle_track[0] = 0.0590396 ;
	// joint_angle_track[1] =  2.17705 ;
	// joint_angle_track[2] = 2.71345 ;
	// joint_angle_track[3] = 0.581935 ;
	// joint_angle_track[4] = 0.0;

	joint_angle_track[0] =  -1.378721 ;
	joint_angle_track[1] =  1.34551 ;
	joint_angle_track[2] =  0.609537 ;
	joint_angle_track[3] =  -1.9422 ;
	joint_angle_track[4] =  0.203068;

	/* Horizontal Waypoint for the trajectory */
	waypoint_h.resize(5);
	
  
  
 
 
 
 


	waypoint_h(0) = -0.461134; 
	waypoint_h(1) =  2.02762; 
	waypoint_h(2) =  2.60264; 
	waypoint_h(3) = 0.697351;
	waypoint_h(4) = 0.0;

	/* Vertical waypoint for the trajectory*/
	
	waypoint_v.resize(5);

	waypoint_v(0) = -0.463527; 
	waypoint_v(1) =  2.09806; 
	waypoint_v(2) = 1.99489; 
	waypoint_v(3) = -1.53725;
	waypoint_v(4) = 0.0687954;

	/* Homing position in Joint Angles*/
	homing.resize(5);
	   
	homing(0) = 0.0499082;
	homing(1) = 2.33726;
	homing(2) = 2.94236;
	homing(3) = 0.729311;
	homing(4) = 0.0;


	last_arm_hold_pos.resize(NUM_JOINTS); 
	for(int i=0; i<NUM_JOINTS; i++){
		last_arm_hold_pos(i) = homing(i);
	}
	/**/ 
	tf::Quaternion in_q = tf::createQuaternionFromRPY(0,M_PI/2,0); // x-vertical - point staight to the device
	orient_h.x = in_q.x();orient_h.y = in_q.y();
	orient_h.z = in_q.z();orient_h.w = in_q.w();

	tf::Quaternion in_q_v = tf::createQuaternionFromRPY(0,M_PI,M_PI/2); // x-vertical - point staight to the device
	orient_v.x = in_q_v.x();orient_v.y = in_q_v.y();
	orient_v.z = in_q_v.z();orient_v.w = in_q_v.w();


	//KDL::Vector linear(0,0,0);
	//KDL::Vector angular(0.349066, 0.349066, 0.349066);
	KDL::Vector angular(0.05, 0.05, 3.14);
	tolerances_h.vel = KDL::Vector(0.05,0,0); tolerances_h.rot = angular;
	tolerances_v.vel = KDL::Vector(0.0,0, 0.05);
	tolerances_v.rot = KDL::Vector(0.05, 0.05, 3.14);
}


KDL::Frame motion::getFK(KDL::JntArray joints){
	KDL::ChainFkSolverPos_recursive fk_solver(chain);
	std::cout<<joints(0)<<" " << joints(1)<<" " << joints(2)<<" " << joints(4) << joints(5)<<endl;
	KDL::Frame pose;
	fk_solver.JntToCart(joints,pose);
	double x,y,z,w;
	std::cout<<pose.p.x() <<  " " << pose.p.y() << " " << pose.p.z() <<endl;
	
	pose.M.GetQuaternion(x,y,z,w); 
	std::cout<< x << " " << y << " " <<z << " " <<w<<endl;
	return(pose);
}

KDL::JntArray motion::getIK(geometry_msgs::Pose target_pose, int dev_orient)
{
	//valid = tracik_solver->getKDLChain(chain);
	KDL::Vector posit(target_pose.position.x, target_pose.position.y, target_pose.position.z);
	tf::Quaternion in_q = tf::createQuaternionFromRPY(M_PI / 2, 0, 0);
	KDL::Rotation orient = KDL::Rotation::Quaternion(in_q.x(), in_q.y(), in_q.z(), in_q.w());

	KDL::Frame end_effector_pose(orient,posit);
	KDL::JntArray nominal(chain.getNrOfJoints());
  
	for (uint j=0; j<nominal.data.size(); j++ ) {
		nominal(j) = joint_angle_track[j];
	}
	int rc; 
	switch(dev_orient){
		case 0 : {
			ROS_INFO("looking down");
			rc = tracik_solver->CartToJnt(nominal, end_effector_pose, result, tolerances_v);
			break;
		}
		case 1: {
			rc = tracik_solver->CartToJnt(nominal, end_effector_pose, result, tolerances_h);
			break;
		}

	}
	
	if (rc > 0) {

		ROS_INFO("ik succeed");
		
		for (uint j=0; j<result.data.size(); j++ ) {
		joint_angle_track[j] = result(j);
		std::cout << result(j) <<std::endl;
		}

		
	}
	else {
		ROS_FATAL("ik failed");
		result(0) = nan;
	}
	std::cout << "rc:" << rc << std::endl;
	switch(dev_orient){
	case 0 : {

		result(3) = -1 * (abs(result(1)) - abs(result(2))) - 1.57 + 0.174533;
		break; 
						}
	case 1:
						{
		result(3) = -1 * (abs(result(1)) - abs(result(2))) + 0.174533;
		break;
						}
	}
	return(result);
}

/* MOTION: to_homing */
bool motion::to_homing(){ 
	ROS_INFO("Homing the robot"); 
	//const double nan = std::numeric_limits<float>::quiet_NaN();
	Eigen::VectorXd current_position = hebi_feedback_arm();
	int num_points=3;
	//jammer_rot = 0 ;
	Eigen::MatrixXd positions(NUM_JOINTS, num_points);
	Eigen::MatrixXd velocities(NUM_JOINTS, num_points);
	Eigen::MatrixXd accelerations(NUM_JOINTS, num_points);

	for (int i = 0; i < NUM_JOINTS; i++)
	{
		positions.block(i, 0, 1, num_points) << current_position[i], homing(i), homing(i);
		velocities.block(i, 0, 1, num_points) << 0, nan, 0;
		accelerations.block(i, 0, 1, num_points) << 0, nan, 0;
	}
	
	// The times to reach each waypoint (in seconds)
	Eigen::VectorXd time(num_points);
	time << 0, 5, 6;
	
	bool execute = exec_traj(time,  positions, velocities,  accelerations);
	jammer_rot = 0.0;
	hebi::GroupCommand hand_cmd(group_hand->size());
	Eigen::VectorXd pos_cmd_hand(group_hand->size());
	Eigen::VectorXd vel_cmd_hand(group_hand->size());
	pos_cmd_hand[0] = jammer_rot;
	vel_cmd_hand << .3;
	
	Eigen::VectorXd cur_hand_angle = hebi_feedback_hand();
	cout << pos_cmd_hand << endl;
	hand_cmd.setPosition(pos_cmd_hand);
	hand_cmd.setVelocity(vel_cmd_hand);
	group_hand->sendCommand(hand_cmd);

	while (abs(cur_hand_angle(0) - pos_cmd_hand(0)) > 0.1)
	{
		ROS_INFO("sending commands ");
		cur_hand_angle = hebi_feedback_hand();
		group_hand->sendCommand(hand_cmd);
	}
	
	at_home = true;
	return execute;
}

bool motion::exec_traj(Eigen::VectorXd time, Eigen::MatrixXd positions, Eigen::MatrixXd velocities, Eigen::MatrixXd accelerations){
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
	Eigen::VectorXd vel_cmd_hand(group_hand->size());
	pos_cmd_hand[0] = jammer_rot;
	vel_cmd_hand << .3;


	/* Break position holding before sending next command */
	reset_hold();

	Eigen::VectorXd cur_hand_angle = hebi_feedback_hand();
	cout << pos_cmd_hand << endl;
	hand_cmd.setPosition(pos_cmd_hand);
	hand_cmd.setVelocity(vel_cmd_hand);
	group_hand->sendCommand(hand_cmd);

	while (abs(cur_hand_angle(0) - pos_cmd_hand(0)) > 0.1)
	{
		ROS_INFO("sending commands ");
		cur_hand_angle = hebi_feedback_hand();
		group_hand->sendCommand(hand_cmd);
	}
	/* Execution of trajectory is blocking */
	for (double t = 0; t < duration; t += period)
	{
		// Pass "nullptr" in to ignore a term.
		trajectory->getState(t, &pos_cmd, &vel_cmd, nullptr);
		cmd.setPosition(pos_cmd);
		cmd.setVelocity(vel_cmd);
		group->sendCommand(cmd);
		group_hand->sendCommand(hand_cmd);
		std::this_thread::sleep_for(
				std::chrono::milliseconds((long int)(period * 1000)));
	}
	at_home = false;
	
	// after last iteration, the cmd should be hold target
	set_hold(pos_cmd);
	last_arm_hold_pos = pos_cmd;
	/* Hold position in background */
	std::thread t([pos_cmd, pos_cmd_hand, period, this]() {
		ROS_INFO("hold position thread");
		while (should_hold_pos())
		{
			hebi::GroupCommand hold_cmd(NUM_JOINTS);
			hold_cmd.setPosition(arm_hold_pos_target);
			group->sendCommand(hold_cmd);
			// std::this_thread::sleep_for(
			// 		std::chrono::milliseconds((long int)(period * 1000)));
		}
		ROS_INFO("hold position loop end");
	});

	t.detach();
	// if(!at_home){
	// 	homed = to_homing();
	// }
	return true;
}

/* MOTION: exec_arm */
bool motion::exec_arm(geometry_msgs::Pose target_pose, int init_rot, int device_orient, bool hold)
{

	// Define nan variable for readability below
	ROS_INFO("Executing Arm\n");
	//const double nan = std::numeric_limits<float>::quiet_NaN();
	int num_points=3;
	Eigen::VectorXd waypoint;
	switch (device_orient)
	{
	case 0:
	{
		//target_pose.orientation = orient_v;
		waypoint = waypoint_v;
		break; 
		}
		case 1: {
			
			//target_pose.orientation = orient_h; 
			waypoint = waypoint_h; 
			break;
		}
	}
	KDL::JntArray target_position = getIK(target_pose, device_orient);
	if(target_position(0) == nan){ 
		return false;
	}
	bool homed;

	
	if(!at_home){
		ROS_INFO("Homing");
		jammer_rot=0;
		homed = to_homing();
		ROS_INFO("Finshed to_homing\n");
	}
	Eigen::VectorXd current_position = hebi_feedback_arm();
	
	Eigen::MatrixXd positions(NUM_JOINTS,num_points);
	Eigen::MatrixXd velocities(NUM_JOINTS,num_points);
	Eigen::MatrixXd accelerations(NUM_JOINTS,num_points);
	jammer_rot = init_rot * M_PI / 180;
	for (int i = 0; i < NUM_JOINTS; i++)
	{
		positions.block(i, 0, 1, num_points) << last_arm_hold_pos[i], waypoint(i), target_position(i);
		velocities.block(i, 0, 1, num_points) << 0, nan, 0;
		accelerations.block(i, 0, 1, num_points) << 0, nan, 0;
	}

	
			// The times to reach each waypoint (in seconds)
	Eigen::VectorXd time(num_points);
	// time << 0, 15, 25, 33;
	time << 0, 10, 20;

	bool execute = exec_traj(time, positions, velocities, accelerations);
	Eigen::VectorXd last_correction = hebi_feedback_arm();
	double elbow_3; 
	// Last Elbow 3 correction
	switch (device_orient)
	{
	case 0:
	{
		elbow_3 = -1 * (abs(last_correction(1)) - abs(last_correction(2))) - 1.57 + 0.174533;
		break;
	}
	case 1:
	{
		elbow_3 = -1 * (abs(last_correction(1)) - abs(last_correction(2))) + 0.174533;
		break;
	}
	}
	ROS_INFO("Last correction");
	Eigen::MatrixXd correct_pos(NUM_JOINTS, 2), correct_vel(NUM_JOINTS,2), correct_accel(NUM_JOINTS,2); 
	Eigen::VectorXd correct_time(2);
	correct_time<<0,1;
	cout<<"here"<<endl;
	for (int i=0; i<NUM_JOINTS; i++){
		correct_pos.block(i,0,1,2)<<last_arm_hold_pos(i), last_arm_hold_pos(i);
		correct_vel.block(i,0,1,2) << 0,0;
		correct_accel.block(i,0,1,2)<< 0,0;
	}
	cout<<"hello"<<endl;
	correct_pos(3,1) = elbow_3;
	bool execute_correction = exec_traj(correct_time, correct_pos, correct_vel, correct_accel);

	return execute_correction;
}

/* MOTION: exec_hand */
bool motion::exec_hand(int rotate, float delta_z){
	ROS_INFO("Executing hand");
	Eigen::VectorXd current_pos = hebi_feedback_arm();
	double jam_angle = rotate *M_PI/180;
	jammer_rot = jam_angle;
	double period  = 0.01;
	Eigen::VectorXd pos(NUM_JOINTS);
	Eigen::VectorXd pos_cmd_hand(group_hand->size());
	Eigen::VectorXd eff_cmd(group_hand->size());

	//Eigen::MatrixXd pos(NUM_JOINTS, 2), vel(NUM_JOINTS,2), accel(NUM_JOINTS, 2); 
	eff_cmd<<.3;
	cout<<(delta_z == 0)<<endl;
	cout << delta_z << endl;
	if(delta_z == 0){ 
		ROS_INFO("Rotating hand");
		pos_cmd_hand<<jam_angle;
	
	}
	else{ 
		ROS_INFO("Just execute z correction");
		target_pose.position.z+= delta_z;
		cout<<target_pose.position<<endl; 
		cout<<target_pose.orientation<<endl;
		exec_correction(target_pose, 0);
		return true;
		
	}

	// reset_hold();
	
	hebi::GroupCommand hand_cmd(group_hand->size());
	Eigen::VectorXd cur_hand_angle = hebi_feedback_hand();
	cout<<pos_cmd_hand<<endl;
	hand_cmd.setPosition(pos_cmd_hand); 
	hand_cmd.setVelocity(eff_cmd);
	group_hand->sendCommand(hand_cmd);

	
	while(abs(cur_hand_angle(0) - pos_cmd_hand(0)) > 0.1) {
		ROS_INFO("sending commands ");
		cur_hand_angle = hebi_feedback_hand();
		group_hand->sendCommand(hand_cmd); 
		
	}

	ROS_INFO("Sending to homing ");
	bool homed = to_homing();
	
	//set_hold();
	return true;
}

/* MOTION: exec_correction */
bool motion::exec_correction(geometry_msgs::Pose corrected_pose, float y_degrees){ 
	ROS_INFO("Correcting end-effector");
	Eigen::VectorXd current_pos = hebi_feedback_arm();
	double period  = 0.01;
	double y_rad = current_pos(0) + y_degrees*M_PI/180;
	//Eigen::VectorXd pos(NUM_JOINTS);
	Eigen::MatrixXd positions(NUM_JOINTS, 2), velocities(NUM_JOINTS, 2), accelerations(NUM_JOINTS, 2);
	KDL::JntArray pos_ = getIK(corrected_pose, device_orient);
	if (pos_(0) == nan)
	{
		return false;
	}
	for(int i=0; i< NUM_JOINTS; i++){
		positions.block(i, 0, 1, 2) << last_arm_hold_pos[i], pos_(i);
		velocities.block(i, 0, 1, 2) << 0,0;
		accelerations.block(i, 0, 1, 2) << 0,0;
	}
	positions(NUM_JOINTS-1, 1) = current_pos[NUM_JOINTS-1];
	positions(0,1) = y_rad;
	Eigen::VectorXd time(2); 
	time<<0,2;

	bool execute = exec_traj(time, positions, velocities, accelerations);

	Eigen::VectorXd last_correction = hebi_feedback_arm();
	double elbow_3;
	// Last Elbow 3 correction
	switch (device_orient)
	{
	case 0:
	{
		elbow_3 = -1 * (abs(last_correction(1)) - abs(last_correction(2))) - 1.57 + 0.174533;
		break;
	}
	case 1:
	{
		elbow_3 = -1 * (abs(last_correction(1)) - abs(last_correction(2))) + 0.174533;
		break;
	}
	}
	ROS_INFO("Last correction");
	Eigen::MatrixXd correct_pos(NUM_JOINTS, 2), correct_vel(NUM_JOINTS, 2), correct_accel(NUM_JOINTS, 2);
	Eigen::VectorXd correct_time(2);
	correct_time << 0, 1;
	for (int i = 0; i < NUM_JOINTS; i++)
	{
		correct_pos.block(i, 0, 1, 2) << last_arm_hold_pos(i), last_arm_hold_pos(i);
		correct_vel.block(i, 0, 1, 2) << 0, 0;
		correct_accel.block(i, 0, 1, 2) << 0, 0;
	}
	correct_pos(3, 1) = elbow_3;
	bool execute_correction = exec_traj(correct_time, correct_pos, correct_vel, correct_accel);

	return execute_correction;
}

void motion::reset_hold(void) {
	ROS_INFO("reset_hold()");
	should_hold_position = false;
	std::this_thread::sleep_for(
		std::chrono::milliseconds((long int)(500)));
}

void motion::set_hold(Eigen::VectorXd target) {
	ROS_INFO("set_hold()");
	arm_hold_pos_target = target;
	should_hold_position = true;
}

bool motion::should_hold_pos(void) {
	return should_hold_position;
}


/* Get back positions of the hand*/
Eigen::VectorXd motion::hebi_feedback_hand(){ 
	group_hand->setFeedbackFrequencyHz(10);
	GroupFeedback group_fbk_hand(group_hand->size());
	if ( group_hand->getNextFeedback(group_fbk_hand))
	{
		std::cout << "Got feedback From hand. Positions are: " << std::endl << group_fbk_hand.getPosition() << std::endl;
		return(group_fbk_hand.getPosition());
	}
	group_hand->clearFeedbackHandlers();
}

/* Get back positions of the arm */
Eigen::VectorXd motion::hebi_feedback_arm(){ 
	group->setFeedbackFrequencyHz(10);
	GroupFeedback group_fbk(group->size());
	if (group->getNextFeedback(group_fbk) )
	{
		std::cout << "Got feedback. Positions are: " << std::endl << group_fbk.getPosition() << std::endl;
		return(group_fbk.getPosition());
	}
	group->clearFeedbackHandlers();
	
}
