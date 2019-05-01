#include <ros/ros.h>

#include "kinematics.h"

using namespace std;

int main(int argc, char **argv)
{
	/* This node is for experiments. All service calls in final integration
	 * are located in jr_kinematics_node.cpp
	 */
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

	target.position.x = 0.46;
	target.position.y = 0.0;
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