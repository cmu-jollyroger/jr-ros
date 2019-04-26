#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <boost/date_time.hpp>
#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <chrono>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include "hebi_cpp_api/lookup.hpp"
#include "hebi_cpp_api/group_command.hpp"
#include "hebi_cpp_api/group_feedback.hpp"
#include <vector>
#include <Eigen/Dense>
#include "jr_common.h"
#include "hebi_cpp_api/trajectory.hpp"
// #include "hebi_cpp_api/util/grav_comp.hpp"
//#include <robot_state_publisher/robot_state_publisher.h>
using namespace std;
using namespace hebi;

class motion
{
public:
	motion();
	KDL::JntArray getIK(geometry_msgs::Pose target_pose);
	bool exec_traj(geometry_msgs::Pose target_pose, int d_orient, int device);
	//Eigen::MatrixXd interpolateJnts(Eigen::VectorXd cpos, Eigen::VectorXd tpos, int num_wayp);

	bool hebi_send_command(Eigen::VectorXd pos);
	Eigen::VectorXd hebi_feedback();

	ros::Publisher joint_pub;
	Eigen::VectorXd homing;

private: 
	KDL::Chain chain;
	std::string chain_start = "base_link";
	std::string chain_end = "end_eff";
	std::string urdf_param = "/robot_description";
	double timeout = 0.06;
	double eps = 100;
	KDL::JntArray result;
	KDL::JntArray ll, ul; 

  	
	TRAC_IK::TRAC_IK *tracik_solver;
	TRAC_IK::SolveType type=TRAC_IK::Distance;
	ros::NodeHandle nh;
	KDL::Tree my_tree;
	std::string robot_desc_string;
	std::vector<double> joint_angle_track;
	

	Lookup lookup;
	std::shared_ptr<Group> group;
	


};


motion::motion(){



	group = lookup.getGroupFromNames({ "JollyRoger Arm" }, {"Base", "Elbow-1", "Elbow-2", "EndEffector"});
	if (!group)
	{
	std::cout << "Group not found!" << std::endl;
	exit;
	}
	group->setCommandLifetimeMs(1000);
	joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states",10);
	ll.resize(5); ul.resize(5);
	
	ll(0)=-3.14 ; ul(0)=3.14;
	ll(1)=-3.14 ; ul(1)=3.14;
	ll(2)=-3.14 ; ul(2)=3.14;
	ll(3)=-3.14 ; ul(3)=3.14;
	ll(4)=-3.14 ; ul(4)=3.14;

	
	nh.param("/robot_description", robot_desc_string, std::string());
	if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
		ROS_ERROR("Failed to construct kdl tree");
	}
	bool got_chain = my_tree.getChain(chain_start,chain_end,chain);
	if(!got_chain){
		ROS_INFO("Unable to load KDL chain");
	}
	tracik_solver = new TRAC_IK::TRAC_IK(chain,ll,ul,timeout, 100,type=type);
	bool valid = tracik_solver->getKDLChain(chain);

  	//std::cout << "num of joints in chain: " << chain.getNrOfJoints() << std::endl;


	joint_angle_track.resize(5);
	joint_angle_track[0] = 0.00909102 ;
	joint_angle_track[1] =-0.411375 ;
	joint_angle_track[2] = 2.74059 ;
	joint_angle_track[3] = 7.90925 ;
	joint_angle_track[4] = 0.0;



	homing.resize(5);




	homing(0) = 0.01647687 ;
	homing(1) =0.68725264 ;
	homing(2) =2.90794563 ;
	homing(3) = 7.90925 ;
	homing(4) = 6.95029498;
}


KDL::JntArray motion::getIK(geometry_msgs::Pose target_pose){ 
	
  	// valid = tracik_solver->getKDLChain(chain);
	KDL::ChainFkSolverPos_recursive fk_solver(chain);
	KDL::JntArray jnts; 
	jnts.resize(5);
	jnts(0) = 0.00909102 ;
	jnts(1) =-0.411375 ;
	jnts(2) = 2.74059 ;
	jnts(3) = 7.90925 ;
	jnts(4) = 0.0;
	
	KDL::Frame pose;
	fk_solver.JntToCart(jnts,pose);
	double x,y,z,w;

	cout<<pose.p.x() << pose.p.y() <<pose.p.z() <<endl;
	pose.M.GetQuaternion(x,y,z,w); 
	cout<< x << y <<z <<w<<endl;
 
  	KDL::Vector posit(target_pose.position.x, target_pose.position.y, target_pose.position.z);
  	KDL::Rotation orient = KDL::Rotation::Quaternion(target_pose.orientation.x, target_pose.orientation.y,
  													 target_pose.orientation.z, target_pose.orientation.w);

  	KDL::Frame end_effector_pose(orient,posit);
	KDL::JntArray nominal(chain.getNrOfJoints());
  
	for (uint j=0; j<nominal.data.size(); j++ ) {
		nominal(j) = joint_angle_track[j];
	}

	int rc=tracik_solver->CartToJnt(nominal,end_effector_pose,result);
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


Eigen::MatrixXd v1_traj(DeviceOrientation orien, geometry_msgs::Pose target_pose){ 
	motion ik;
	//Eigen::VectorXd curr_jnts = ik.hebi_feedback();
	Eigen::VectorXd curr_jnts; curr_jnts.resize(4);
	curr_jnts<< 0,
				0,
				0,
				0;
	Eigen::VectorXd target_jnts;target_jnts.resize(4);
	
	tf::Quaternion in_q; int num;
	switch(orien){
		case VERTICAL: in_q= tf::Quaternion(-0.491329,0.487592,0.513436,0.507182);num = 3;
						break;
		case HORIZONTAL: in_q= tf::Quaternion(tf::createQuaternionFromRPY(0,M_PI,M_PI/2));num = 3;
						break;
	}
	target_pose.orientation.x = in_q.x();target_pose.orientation.y = in_q.y();
	target_pose.orientation.z = in_q.z();target_pose.orientation.w = in_q.w();
	
	Eigen::MatrixXd traj(4,5);
	
	KDL::JntArray tresult = ik.getIK(target_pose);
	KDL::JntArray mresult;
	if (num ==2){
		printf("num_2\n");
		traj<<curr_jnts(0), ik.homing(0), tresult(0), ik.homing(0),ik.homing(0),
			  curr_jnts(1), ik.homing(1), tresult(1), ik.homing(1),ik.homing(1),
			  curr_jnts(2), ik.homing(2), tresult(2), ik.homing(2),ik.homing(2),
			  curr_jnts(3), ik.homing(3), tresult(3), ik.homing(3),ik.homing(3);
			//  curr_jnts(4), tresult(4);
	}
	
	else if (num ==3){
		printf("num_2\n");
		target_pose.position.y+=0.05;		
		mresult = ik.getIK(target_pose);
		traj<<curr_jnts(0), ik.homing(0), mresult(0), tresult(0), ik.homing(0),
			  curr_jnts(1), ik.homing(1), mresult(1), tresult(1), ik.homing(1),
			  curr_jnts(2), ik.homing(2), mresult(2), tresult(2), ik.homing(2),
			  curr_jnts(3), ik.homing(3), mresult(3), tresult(3), ik.homing(3);
			//  curr_jnts(4), mresult(4), tresult(4);
	}

	cout<<traj<<endl;
	printf("done\n");
	return(traj);

	
}
Eigen::MatrixXd v2_traj(geometry_msgs::Pose target_pose){ 
	motion ik;
	// Eigen::VectorXd curr_jnts = hebi_feedback();
	Eigen::VectorXd curr_jnts; curr_jnts.resize(4);
	curr_jnts<< 0,
				0,
				0,
				0;
	Eigen::VectorXd target_jnts;target_jnts.resize(4);
	tf::Quaternion in_q;
	in_q = tf::Quaternion(-0.491329,0.487592,0.513436,0.507182);

	target_pose.orientation.x = in_q.x();target_pose.orientation.y = in_q.y();
	target_pose.orientation.z = in_q.z();target_pose.orientation.w = in_q.w();
	
	Eigen::MatrixXd traj(4,5);
	
	KDL::JntArray tresult = ik.getIK(target_pose);
	//KDL::JntArray mresult;
	
	printf("num_2\n");
	traj<<curr_jnts(0), ik.homing(0), tresult(0), ik.homing(0),ik.homing(0),
		  curr_jnts(1), ik.homing(1), tresult(1), ik.homing(1),ik.homing(1),
		  curr_jnts(2), ik.homing(2), tresult(2), ik.homing(2),ik.homing(2),
		  curr_jnts(3), ik.homing(3), tresult(3), ik.homing(3),ik.homing(3);
		//  curr_jnts(4), tresult(4);

	cout<<traj<<endl;
	printf("done\n");
	return(traj);

}
// Eigen::MatrixXd v3_traj(DeviceOrientation orien, geometry_msgs::Pose target_pose){ 
// 	// Eigen::VectorXd curr_jnts = hebi_feedback();
// 	Eigen::VectorXd target_jnts; target_jnts.resize(5);
// 	tf::Quaternion in_q;
// 	switch(orien){
// 		case VERTICAL: in_q= tf::Quaternion(-0.491329,0.487592,0.513436,0.507182);
// 		case HORIZONTAL: in_q= tf::Quaternion(tf::createQuaternionFromRPY(0,M_PI,M_PI/2));
// 	}
// 	target_pose.orientation.x = in_q.x();target_pose.orientation.y = in_q.y();
// 	target_pose.orientation.z = in_q.z();target_pose.orientation.w = in_q.w();
// 	motion ik;
// 	KDL::JntArray result = ik.getIK(target_pose);
// 	for(uint i=0; i<result.data.size(); i++){
// 		target_jnts(i) = result(i);
// 	}


Eigen::MatrixXd b_traj(DeviceOrientation orien, geometry_msgs::Pose target_pose ){ 
	motion ik;
	// Eigen::VectorXd curr_jnts = hebi_feedback();
	Eigen::VectorXd curr_jnts; curr_jnts.resize(4);
	curr_jnts<< 0,
				0,
				0,
				0;
	Eigen::VectorXd target_jnts; target_jnts.resize(5);
	tf::Quaternion in_q(-0.491329,0.487592,0.513436,0.507182);
	target_pose.orientation.x = in_q.x();target_pose.orientation.y = in_q.y();
	target_pose.orientation.z = in_q.z();target_pose.orientation.w = in_q.w();
	geometry_msgs::Pose move_up = target_pose;
	switch(orien){
		case VERTICAL: move_up.position.z+=0.1; break;
		case HORIZONTAL: move_up.position.z-=0.1; break;
	}
	

	Eigen::MatrixXd traj(4,5);
	KDL::JntArray tresult = ik.getIK(target_pose);
	KDL::JntArray mresult = ik.getIK(move_up);
	traj<<curr_jnts(0), ik.homing(0), tresult(0), mresult(0),ik.homing(0),
		  curr_jnts(1), ik.homing(1), tresult(1), mresult(1),ik.homing(1),
		  curr_jnts(2), ik.homing(2), tresult(2), mresult(2),ik.homing(2),
		  curr_jnts(3), ik.homing(3), tresult(3), mresult(3),ik.homing(3);
	cout<<traj<<endl;
	printf("done\n");
	return(traj);
}


bool motion::exec_traj(geometry_msgs::Pose target_pose, int d_orient, int dev){

	// get_current_pose 
	Eigen::VectorXd curr_jnts = hebi_feedback();

	
	const double nan = std::numeric_limits<float>::quiet_NaN();
	// DeviceType device = (enum DeviceType) dev;
	// DeviceOrientation orien = (enum DeviceOrientation)d_orient; 
	// Eigen::MatrixXd traj(4,3); 
	// 
	// switch(device){
	// 	case V1:printf("v1\n");
	// 			traj = v1_traj(orien, target_pose); 
	// 			break;
	// 	case V2: traj = v2_traj(target_pose);break;
	// 	case V3: traj = v1_traj(orien,target_pose);break;
	// 	case B: traj = b_traj(orien,target_pose);break;
	// }

	// printf("vel_acc\n");
	Eigen::MatrixXd positions(4,4);
	Eigen::MatrixXd velocities(4,4);
	Eigen::MatrixXd accelerations(4,4);
	printf("this\n");
	velocities<< 0, nan, nan, 0,
              0, nan, nan, 0,
              0, nan, nan, 0,
              0, nan, nan, 0;
	printf("yo\n");
	positions <<curr_jnts(0), homing(0), 0.09047222 ,  1.35895622,
			  curr_jnts(1), homing(1),0.11336315 ,0.16411419,
			  curr_jnts(2), homing(2),1.8573246 ,1.80936718,
			  curr_jnts(3), homing(3),6.50338507, 6.28021574;
	
	
    printf("yo_2\n");
	accelerations <<0, nan, nan, 0,
                0, nan, nan, 0,
                0, nan, nan, 0,
                0, nan, nan, 0;

	printf("Time\n");
	//get target pose
    Eigen::VectorXd time(4);
	time << 0, 10,20,25;
	int num_joints = 4;
	printf("traj\n");
	auto trajectory = trajectory::Trajectory::createUnconstrainedQp(time, positions, &velocities, &accelerations);

	hebi::GroupCommand cmd(num_joints);
	double period = 0.01;
	double duration = trajectory->getDuration();
	Eigen::VectorXd pos_cmd(num_joints);
	Eigen::VectorXd vel_cmd(num_joints);

	Eigen::VectorXd ss(4);
	ss<<homing(0),
homing(1),
homing(2),
homing(3);


	for (double t = 0; t < duration; t += period)
	{
	  // Pass "nullptr" in to ignore a term.
	  trajectory->getState(t, &pos_cmd, &vel_cmd, nullptr);
	  cmd.setPosition(pos_cmd);
	  cmd.setVelocity(vel_cmd);
	  group->sendCommand(cmd);
	  std::this_thread::sleep_for(std::chrono::milliseconds((long int) (period * 1000)));
	}
	
	while(true){
		hebi_send_command(ss);
	}

	return true;


}

// Eigen::MatrixXd motion::interpolateJnts(Eigen::VectorXd cpos, Eigen::VectorXd tpos, int num_wayp){ 

// 	for(int i=0; i<cpos.size(); i++ ){

// 		double step_size = (tpos(i) - cpos(i))/2;
// 	}



// }


bool motion::hebi_send_command( Eigen::VectorXd pos){
	
	Eigen::VectorXd efforts(group->size());
	
	efforts.setZero();
	

	hebi::GroupCommand groupCommand(group->size());

	groupCommand.setEffort(efforts);
	groupCommand.setPosition(pos);
	group->sendCommand(groupCommand);
	return true;
}



Eigen::VectorXd motion::hebi_feedback(){ 

	// This is by default "100"; setting this to 5 here allows the console output
	// to be more reasonable.
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
	
	motion ik; 
	//Eigen::VectorXd position; position.resize(4); 


   
   
	// position<<  0.111384,
	// -0.233068,
	//  2.63529,
 //  	1.57;
	ik.hebi_feedback();
	// while(ros::ok()){
	// 	ik.hebi_send_command(position);
	// 	ros::spinOnce();
	// }
	//cout<<position<<endl;
	geometry_msgs::Pose target; 
	bool res =ik.exec_traj(target,0,1);

	// geometry_msgs::Pose target; 
	// //target.position.x =0.03454648;
	// target.position.x =0.0;
	// target.position.y =0.35;
	// target.position.z = 0.49;
	// tf::Quaternion in_q(-0.491329,0.487592,0.513436,0.507182);  // vertical
	// // tf::Quaternion in_q = tf::createQuaternionFromRPY(0,M_PI,M_PI/2); // horizontal

	// // target.orientation.x = in_q.x();target.orientation.y = in_q.y();
	// // target.orientation.z = in_q.z();target.orientation.w = in_q.w();
	// printf("getIk\n");
	// KDL::JntArray result = ik.getIK(target);
	// //bool res = ik.exec_traj(target,0,1);
	// printf("Got here_1\n");
	// tf::TransformBroadcaster br;
 //  	tf::Transform transform;
 //  	transform.setOrigin( tf::Vector3(target.position.x, target.position.y, target.position.z) );
 //  	transform.setRotation(in_q);
  	
 //  	printf("Got here_2\n");
  

 //  	sensor_msgs::JointState joint_state;
 //  	joint_state.position.resize(5);
 //  	joint_state.name.resize(5);
 //  	joint_state.name[0] = "dummy_to_base";
 //    joint_state.name[1] = "dummy_to_elbow_1";
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