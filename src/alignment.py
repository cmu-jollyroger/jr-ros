import numpy as np 
import time 
import math
import rospy
from geometry_msgs.msg import Twist,Pose
import copy 

pub_pos = rospy.Publisher('estimate_pose', Pose, queue_size=10)

center_offset = np.array([209.55,254,279.4,279.4,254,209.55])
tof_angles = np.array([np.deg2rad(180), np.deg2rad(144), np.deg2rad(108), 
                       np.deg2rad(72), np.deg2rad(36), np.deg2rad(0)])

def init(data):
	tof_d = np.array([data.linear.x, data.linear.y, data.linear.z, 
	 				  data.angular.x, data.angular.y, data.angular.z])

	x = (tof_d[0] + center_offset[0])/1000
	y_ = tof_d[2] + center_offset[2]
	y = (y_*math.cos(tof_angles[2]) -59.69)/1000
	print(x,y)
	p = Pose() 
	p.position.x = x
	p.position.y = y
	pub_pos.publish(p)


def hitlimit(data,arg):
	arg = data.sw_1 & data.sw_2	

def getdata(data,arg): 
	arg = [data.position.x, data.position.y]
	

if __name__ == '__main__':
	rospy.init_node('tof_localize', anonymous=True)
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

	lim1_lim2 = False
	#rospy.Subscriber("tof_readings", Twist, init)
	t_end = time.time() + 9
	vel = 2
	rospy.Subscriber("chassis_info", add chassis datatype, hitlimit, (lim1_lim2))
	while(time.time() < t_end and not lim1_lim2): 
		rospy.Subscriber("chassis_info", add chassis datatype, hitlimit,(lim1_lim2))
		v = Twist()
		v.linear.x = vel 
		pub.publish(v)
	t_back = time.time() +4
	while( time.time() < t_back): 
		b = Twist()
		b.linear.x = -0.5
		pub.publish(b)

	
	s = Twist()
	s.linear.x = 0.0
	pub.publish(s)

	# test till here first if we can stop when limit switches are triggered
	x_y = [0,0]
	rospy.Subscriber("tof_readings", Twist, init)
	rospy.Subscriber('estimate_pose', Pose, getdata,(x_y))

	target_pose = copy.deepcopy(x_y)
	target_pose[1] += 0.10

	if abs(target_pose[1] - x_y[1])<0.1: 
		b = Twist()
		b.linear.x = 0.5
		pub.publish(b)
		rospy.Subscriber("tof_readings", Twist, init)
		rospy.Subscriber('estimate_pose', Pose, getdata,(x_y))



	#rospy.Subscriber("tof_readings", Twist, init)
	rospy.spin()
	print("done")
	#rospy.spin()
