

	// joint_angle_track[0] = 0.00909102 ;
	// joint_angle_track[1] =-0.411375 ;
	// joint_angle_track[2] = 2.74059 ;
	// joint_angle_track[3] = 7.90925 ;
	// joint_angle_track[4] = 0.0;



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

