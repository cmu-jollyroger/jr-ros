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

#define NUM_JOINTS (4)

using namespace hebi;

class motion
{
public:
	/* Constructor, looks up hebi modules */
	motion(std::string robot_desc_string);
	/* Get joint angles from target pose */
	KDL::JntArray getIK(geometry_msgs::Pose target_pose, int dev_orient);
	/* Get pose from joint angles */
	KDL::Frame getFK(KDL::JntArray joints);
	/* Execute Traj */
	bool exec_traj(Eigen::VectorXd time, Eigen::MatrixXd positions, Eigen::MatrixXd velocities, Eigen::MatrixXd accelerations);
	/* Execute arm */
	bool exec_arm(geometry_msgs::Pose target_pose, int init_rot, int device_orient, bool hold);
	/* Execute correction */
	bool exec_correction(geometry_msgs::Pose corrected_pose, float y_degrees); 
	/* Execute the latest traj */
	bool exec_hand(int rotate, float delta_z);
	/* Break position holding after traj execution */
	void reset_hold(void);
	/* Set should hold position */
	void set_hold(Eigen::VectorXd target);
	/* Check if should hold position */
	bool should_hold_pos(void);
	/* Goes to homing from current pos */
	bool to_homing();
	/* Reads in current joint angles */
	Eigen::VectorXd hebi_feedback_arm();
	Eigen::VectorXd hebi_feedback_hand();
	
	bool load_groups();

	/* Testing */
	Eigen::VectorXd homing, waypoint_v, waypoint_h;
	geometry_msgs::Quaternion orient_v, orient_h;
	geometry_msgs::Pose target_pose;
	int device_orient;

private: 
	KDL::Chain chain;
	std::string chain_start = "base_link";
	//std::string chain_end_fk = "end_eff";
	std::string chain_end = "elbow_3";

	std::string urdf_param = "/robot_description";
	double timeout = 0.06;
	double eps = 1e-3;
	KDL::JntArray result;
	KDL::JntArray ll, ul;
	KDL::Twist tolerances_v;
	KDL::Twist tolerances_h;

	TRAC_IK::TRAC_IK *tracik_solver;
	TRAC_IK::SolveType type=TRAC_IK::Distance;
	
	KDL::Tree my_tree;
	std::vector<double> joint_angle_track;
	
	Lookup lookup;
	std::shared_ptr<Group> group;
	std::shared_ptr<Group> group_hand;
	
	bool should_hold_position;
	const double nan = std::numeric_limits<float>::quiet_NaN();
	bool at_home = false;
	double jammer_rot;

	Eigen::VectorXd arm_hold_pos_target;
	Eigen::VectorXd last_arm_hold_pos;
};

#endif /* __KINEMATICS_H__ */