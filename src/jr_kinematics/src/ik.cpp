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

motion::motion(std::string robot_desc_string) {
	/* Start the broadcasting to the arm */
	group = lookup.getGroupFromNames(
		{ "JollyRoger Arm" },
		{ "Base", "Elbow-1", "Elbow-2", "EndEffector", "Jammer" }
	);
	if (!group)
	{
		ROS_FATAL("Group not found!");
		exit(-1);
	}
	group->setCommandLifetimeMs(1000);

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

	// waypoint_v.resize(5);
	// waypoint_v(0) = ; 
	// waypoint_v(1) = ; 
	// waypoint_v(2) = ; 
	// waypoint_v(3) = ; 
	// waypoint_v(4) = ; 

	/* Homing position in Joint Angles*/
	homing.resize(5);
	homing(0) = 0.0590396;
	homing(1) =  2.17705;
	homing(2) = 2.71345;
	homing(3) = 0.581935;
	homing(4) = 0.0;

	orient_v = tf::createQuaternionMsgFromRollPitchYaw(0,M_PI,M_PI/2); // looking down
	orient_h = tf::createQuaternionMsgFromRollPitchYaw(0,M_PI/2,0); // looking at device

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

		std::cout << "ik succeed " << std::endl;
		
		for (uint j=0; j<result.data.size(); j++ ) {
		joint_angle_track[j] = result(j);
		std::cout << result(j) <<std::endl;
		}

		
	}
	else {
		std::cout << "ik failed" << std::endl;
	}
	std::cout << "rc:" << rc << std::endl;


	return(result);

}

bool motion::to_homing(){ 
	ROS_INFO("Homing the robot"); 
	const double nan = std::numeric_limits<float>::quiet_NaN();
	Eigen::VectorXd current_position = hebi_feedback();
	int num_joints =5;
	int num_points=3;
	Eigen::MatrixXd positions(num_joints,num_points);
	Eigen::MatrixXd velocities(num_joints,num_points);
	Eigen::MatrixXd accelerations(num_joints,num_points);

	positions << current_position[0], homing(0),homing(0),
	             current_position[1], homing(1),homing(1),
	             current_position[2], homing(2),homing(2),
	             current_position[3], homing(3),homing(3),
	             current_position[4], homing(4),homing(4);

	velocities << 0, nan, 0,
	              0, nan, 0,
	              0, nan, 0,
	              0, nan, 0,
	              0, nan, 0;

	accelerations << 0, nan, 0,
	                 0, nan, 0,
	                 0, nan, 0,
	              	 0, nan, 0,
	              	 0, nan, 0;

	// The times to reach each waypoint (in seconds)
	Eigen::VectorXd time(num_points);
	time << 0, 15, 16;

	// Define trajectory
	auto trajectory = hebi::trajectory::Trajectory::createUnconstrainedQp(time, positions, &velocities, &accelerations);

	hebi::GroupCommand cmd(num_joints);
	double period = 0.01;
	double duration = trajectory->getDuration();
	Eigen::VectorXd pos_cmd(num_joints);
	Eigen::VectorXd vel_cmd(num_joints);
	for (double t = 0; t < duration; t += period)
	{
	  // Pass "nullptr" in to ignore a term.
	  trajectory->getState(t, &pos_cmd, &vel_cmd, nullptr);
	  cmd.setPosition(pos_cmd);
	  cmd.setVelocity(vel_cmd);
	  group->sendCommand(cmd);
	  std::this_thread::sleep_for(std::chrono::milliseconds((long int) (period * 1000)));
	}
		return true;
}

bool motion::exec_traj(geometry_msgs::Pose target_pose, int init_rot, int device_orient, bool hold){

	// Define nan variable for readability below
	ROS_INFO("Executing Traj\n");
	const double nan = std::numeric_limits<float>::quiet_NaN();
	int num_joints =5;
	int num_points=4;
	Eigen::VectorXd waypoint;
	double jammer_rot = init_rot*M_PI/180;
	switch(device_orient){
		case 0 : target_pose.orientation = orient_v; 
				 waypoint = waypoint_v; 
				break; 
		case 1 : target_pose.orientation = orient_h; 
				 waypoint = waypoint_h; 
				break;
	}
	KDL::JntArray target_position = getIK(target_pose);

	bool homed = to_homing();
	ROS_INFO("Finshed to_homing\n");
	Eigen::VectorXd current_position = hebi_feedback();
	
	Eigen::MatrixXd positions(num_joints,num_points);
	Eigen::MatrixXd velocities(num_joints,num_points);
	Eigen::MatrixXd accelerations(num_joints,num_points);


	positions << current_position[0], homing(0), waypoint(0), target_position(0),        
             	 current_position[1], homing(1), waypoint(1), target_position(1),
             	 current_position[2], homing(2), waypoint(2), target_position(2),        
             	 current_position[3], homing(3), waypoint(3), target_position(3),
             	 current_position[4], jammer_rot, jammer_rot, jammer_rot;

	velocities << 0, nan, nan, 0,
                  0, nan, nan, 0,
                  0, nan, nan, 0,
                  0, nan, nan, 0,
                  0, nan, nan, 0;

    accelerations << 0, nan, nan, 0,
                     0, nan, nan, 0,
                     0, nan, nan, 0,
                     0, nan, nan, 0,
                     0, nan, nan, 0;

	// The times to reach each waypoint (in seconds)
	Eigen::VectorXd time(num_points);
	// time << 0, 15, 25, 33;
	time << 0, 3, 20, 33;

	// Define trajectory
	auto trajectory = hebi::trajectory::Trajectory::createUnconstrainedQp(time, positions, &velocities, &accelerations);
	
	// Send trajectory
	hebi::GroupCommand cmd(num_joints);
	double period = 0.01;
	double duration = trajectory->getDuration();
	Eigen::VectorXd pos_cmd(num_joints);
	Eigen::VectorXd vel_cmd(num_joints);

	/* Break position holding before sending next command */
	break_hold();

	/* Execution of trajectory is blocking */
	for (double t = 0; t < duration; t += period)
	{
		// Pass "nullptr" in to ignore a term.
		trajectory->getState(t, &pos_cmd, &vel_cmd, nullptr);
		cmd.setPosition(pos_cmd);
		cmd.setVelocity(vel_cmd);
		group->sendCommand(cmd);
		std::this_thread::sleep_for(
			std::chrono::milliseconds((long int) (period * 1000)));
	}

	// after last iteration, the cmd should be hold target
	if (hold) {
		should_hold_position = true;
	}

	/* Hold position in background */
	std::thread t([&, this](){
		ROS_INFO("hold position thread");
		while (should_hold_position) {
			group->sendCommand(cmd);
			std::this_thread::sleep_for(
				std::chrono::milliseconds((long int) (period * 1000)));
		}
		ROS_INFO("hold position loop end");
	});

	t.detach();
	
	return true;

}

void motion::break_hold(void) {
	ROS_DEBUG("break_hold()");
	should_hold_position = false;
}

// bool motion::hebi_send_command( Eigen::VectorXd pos){
	
// 	Eigen::VectorXd efforts(group->size());
	
// 	efforts.setZero();
	
// 	hebi::GroupCommand groupCommand(group->size());

// 	groupCommand.setEffort(efforts);
// 	groupCommand.setPosition(pos);
// 	group->sendCommand(groupCommand);
// 	return true;
// }



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

int main(int argc, char **argv)
{

	/* code */
	ros::init(argc, argv, "joint_state_publisher");

	ros::start();
	ros::NodeHandle nh;

	ros::Publisher joint_pub;

	std::string robot_desc_string;
	nh.param("/robot_description", robot_desc_string, std::string());

	joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states",10);
	// Check setup
	motion ik(robot_desc_string); 

	geometry_msgs::Pose target; 

	
	
	target.position.x =0.46;
	target.position.y =0.0;
	target.position.z = 0.50;

	/* EXAMPLE 2: XYZ position*/
	// target.position.x =0.03454650;
	// target.position.y =0.252262;
	// target.position.z = 0.112877;

	/* EXAMPLE : Orientation of end effector */

	//tf::Quaternion in_q(-0.491329, 0.487592 ,0.513436 ,0.507182);  // y-vertical - point straight to the side




	//tf::Quaternion in_q = tf::createQuaternionFromRPY(0,M_PI,M_PI/2); // horizontal - point straight down
	tf::Quaternion in_q = tf::createQuaternionFromRPY(0,M_PI/2,0); // x-vertical - point staight to the device
	target.orientation.x = in_q.x();target.orientation.y = in_q.y();
	target.orientation.z = in_q.z();target.orientation.w = in_q.w();
	ROS_INFO("getIk\n");

	/* Get IK */
	KDL::JntArray result = ik.getIK(target);

	/* Execute Trajectory i.e. current pos -> homing -> waypoint -> target position : Comment this 
	bloxk out if you just want to see IK result*/

	bool done = ik.exec_traj(target, 0, 1, true);
	std::cout << done << endl;
	while(1){
		continue;
	}
	/* Uncomment block below to get RVIZ visualization of target pose and IK*/



	// printf("Got here_1\n");
	// tf::TransformBroadcaster br;
 //  	tf::Transform transform;
 //  	transform.setOrigin( tf::Vector3(target.position.x, target.position.y, target.position.z) );
 //  	transform.setRotation(in_q);
  	
 //  	printf("Got here_2\n");
  

 //  	sensor_msgs::JointState joint_state;
 //  	joint_state.position.resize(5);
 //  	joint_state.name.resize(5);
 //  	joint_state.name[0] = "base_to_base";
 //    joint_state.name[1] = "base_to_elbow_1";
 //    joint_state.name[2] = "elbow_1_to_elbow_2";
 //    joint_state.name[3] = "elbow_2_to_elbow_3";
 //    joint_state.name[4] = "elbow_3_to_end_eff";

 //    printf("Got here_3\n");
 //    for(int i=0; i<result.data.size(); i++){
 //    	joint_state.position[i] = result(i);
 //    }

 //    ros::Rate loop_rate(10);
 //    printf("Got here_4\n");
 //    //command the robot to move

 //    while (ros::ok()){
 //    	joint_state.header.stamp = ros::Time::now();
 //    	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "pose"));
 //    	ik.joint_pub.publish(joint_state);
 //    	ros::spinOnce();
	//     loop_rate.sleep();
 //    } 


	// ros::spin();
	return 0;
	
}