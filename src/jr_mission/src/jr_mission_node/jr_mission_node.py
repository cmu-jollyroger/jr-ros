#!/usr/bin/env python

import sys
import copy
import time
import rospy
from jr_actuation.srv import *
from jr_kinematics.srv import *
from jr_device.srv import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist

#format vertical/horizontal current position rest of stuff
# vertical = 0, horizontal = 1
# aligned shuttlecock = 0 (open), not aligned to pipes = 1 (close)
# breaker up = 1, down = 0

#filename = sys.argv[-1]

if __name__ == '__main__':
	# Initialize the node and name it.
	#rospy.init_node('mission', anonymous = True)
	try:
		print "hello world from mission node"
	except rospy.ROSInterruptException:
		print ("Mission Node Error")
		pass

