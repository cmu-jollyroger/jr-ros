#!/usr/bin/env python

import sys
import copy
import time
import rospy
from jr_actuation.srv import *
from jr_kinematics.srv import *


#filename = sys.argv[-1]
def parser():
	filename = "~/Projects/jr-ros/src/jr_mission/src/mission.txt"

	f = open(filename,'r')
	lines = f.readlines()

	for line in lines:
		lineSplit = line.split(", ")
		stations = [x[0] for x in lineSplit[:-1]]

		times = lineSplit[-1]
		lineSplit = [x[1:] for x in lineSplit[:-1]]

		print ("Time for Task " + times)

		for i in range(0, len(lineSplit)):
			task = lineSplit[i]
			tasks = task.split(" ")
			printStatement = "Go to Station " + stations[i]

			rospy.wait_for_service('locomotion_srv')
			try:
				locomotion_srv = rospy.ServiceProxy('locomotion_srv', LocomotionCmd)
				station_value = int(stations[i] - "A")
				result = locomotion_srv(station_value)
				print (result)
			except rospy.ServiceException, e:
				print ("Service call failed: %s"%e)

			#We know it is a Valve
			if (len(tasks) == 2):

				types = tasks[0]
				orientation = int(tasks[1])

				typesNum = 0

				if (types == "V1")
					typesNum = 0
				elif (types == "V2"):
					typesNum = 1
				elif (types == "V3")
					typesNum = 2
				else:
					typesNum = 3

				rospy.wait_for_service('pose_srv')
				try:
					pose_srv = rospy.ServiceProxy('pose_srv', PoseCmd)
					result = pose_srv(typesNum)
					print (result)
				except rospy.ServiceException, e:
					print ("Service call failed: %s"%e)

				rospy.wait_for_service('arm_srv')
				try:
					arm_srv = rospy.ServiceProxy('arm_srv', ArmCmd)
					types = 
					result = arm_srv(typesNum, orientation, pose_msg, turn_angle)
					print (result)
				except rospy.ServiceException, e:
					print ("Service call failed: %s"%e)

				if (types == "V3"):
					if orientation == 1:
						print (printStatement + " and close shuttlecock valve")
					else:
						print (printStatement + " and open shuttlecock valve")
				else:
					print (printStatement + " and rotate wheel valve to " + str(orientation) + " degrees")

			else:
				printStatement = printStatement + " For Breaker " + tasks[0]
				for j in range(1, len(tasks), 2):

					switch = tasks[j]
					orientation = tasks[j + 1]
					if (orientation == "U"):
						printStatement = printStatement + " flip switch " + switch[1:] + " up"

					else:
						printStatement = printStatement + " flip switch " + switch[1:] + " down"

					print (printStatement)


	f.close()


if __name__ == '__main__':
	# Initialize the node and name it.
	rospy.init_node('mission', anonymous = True)
	try:
		parser()
	except rospy.ROSInterruptException:
		print ("Mission Node Error")
		pass

