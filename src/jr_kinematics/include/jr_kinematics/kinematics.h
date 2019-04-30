#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include <vector>
#include <Eigen/Dense>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include "hebi_cpp_api/lookup.hpp"
#include "hebi_cpp_api/group_command.hpp"
#include "hebi_cpp_api/group_feedback.hpp"
#include "hebi_cpp_api/trajectory.hpp"

using namespace hebi;

class motion
{
public:
	/* Constructor, looks up hebi modules */
	motion(std::string robot_desc_string);
	/* Get joint angles from target pose */
	KDL::JntArray getIK(geometry_msgs::Pose target_pose);
	/* Get pose from joint angles */
	KDL::Frame getFK(KDL::JntArray joints);
	/* Execute traj */
	bool exec_traj(geometry_msgs::Pose target_pose, int init_rot);
	/* Goes to homing from current pos */
	bool to_homing();
	/* Reads in current joint angles */
	Eigen::VectorXd hebi_feedback();
	
	/* Testing */
	Eigen::VectorXd homing, waypoint;

private: 
	KDL::Chain chain;
	std::string chain_start = "base_link";
	std::string chain_end = "end_eff";
	std::string urdf_param = "/robot_description";
	double timeout = 0.06;
	double eps = 1e-3;
	KDL::JntArray result;
	KDL::JntArray ll, ul;
	KDL::Twist tolerances;
  	
	TRAC_IK::TRAC_IK *tracik_solver;
	TRAC_IK::SolveType type=TRAC_IK::Distance;
	
	KDL::Tree my_tree;
	std::vector<double> joint_angle_track;
	
	Lookup lookup;
	std::shared_ptr<Group> group;
};

#endif /* __KINEMATICS_H__ */