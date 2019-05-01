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
# vertical = 1, horizontal = 0
# aligned shuttlecock = 0 (open), not aligned to pipes = 1 (close)
# breaker up = 1, down = 0

#filename = sys.argv[-1]
def parser():
	filename = "/home/pi/Projects/jr-ros/src/jr_mission/src/mission.txt"

	try:

		f = open(filename,'r')
		lines = f.readlines()

		for line in lines:
			lineSplit = line.split(", ")
			stations = [x[2] for x in lineSplit[:-1]]

			'''
			sortedStations = copy.deepcopy(stations)
			sortedStations.sort()

			tmp = copy.deepcopy(lineSplit)
			#sort based on stations
			for i in range(0, len(stations)):
				index = sortedStations.index(stations[i])
				tmp[index] = lineSplit[i]

			lineSplit = tmp
			stations = sortedStations

			'''
			
			orientations = [x[0] for x in lineSplit[:-1]]
			positions = [x[1] for x in lineSplit[:-1]]
			#stations = [x[2] for x in lineSplit[:-1]]

			times = lineSplit[-1]
			lineSplit = [x[3:] for x in lineSplit[:-1]]

			print ("Time for Task " + times)

			for i in range(0, len(lineSplit)):
				task = lineSplit[i]
				tasks = task.split(" ")
				printStatement = "Go to Station " + stations[i]

				station_value = int(ord(stations[i]) - ord('A'))
				print ("Station")
				print (stations[i])
				print (station_value)

				

				rospy.wait_for_service('jr_locomotion_cmd')
				try:
					locomotion_srv = rospy.ServiceProxy('jr_locomotion_cmd', LocomotionCmd)
					result = locomotion_srv(station_value)
					print (result)
				except rospy.ServiceException, e:
					print ("Service call failed: %s"%e)


				if (len(tasks) >= 2):
					types = tasks[0]
					orientation = 1
					typesNum = 0
					device_id = 1

					if (types == "V1"):
						orientation = int(tasks[1]) % 360
						if (abs(orientation) <= 15):
							print ("Device in acceptable error range")
							continue
						typesNum = 0
					elif (types == "V2"):
						orientation = int(tasks[1]) % 360
						if (abs(orientation) <= 15):
							print ("Device in acceptable error range")
							continue
						typesNum = 1
					elif (types == "V3"):
						orientation = int(tasks[1])
						if (positions[i] == orientation):
								print ("Shuttlecock in correct position")
								continue
						typesNum = 2
					else:
						device_id = int(tasks[1][1:])
						if (tasks[2] == "U"):
							orientation = 0
							if (int(positions[i]) == 1):
								print ("Breaker Switch in correct position")
								continue
						if (tasks[2] == "D"):
							orientation = 1
							if (int(positions[i]) == 0):
								print ("Breaker Switch in correct position")
								continue
						typesNum = 3

					print ("Waiting for Pose Call")

					rospy.wait_for_service('pose_srv')
					result = None
					try:
						pose_srv = rospy.ServiceProxy('pose_srv', PoseCmd)
						result = pose_srv(typesNum, device_id, station_value)
						print (result)
					except rospy.ServiceException, e:
						print ("Service call failed: %s"%e)


					'''

					if (types == "V3"):
						if (result.dev_ori == int(tasks[1])):
							print ("Shuttlecock in correct position already")
							continue
					'''

					time.sleep(24)


					print ("Waiting for Arm Call")
					'''
					rospy.wait_for_service('arm_srv')
					try:
						arm_srv = rospy.ServiceProxy('arm_srv', ArmCmd)
						orientation = orientations[i]
						result = arm_srv(typesNum, result.dev_ori, result.pose, orientation)
						print (result)
					except rospy.ServiceException, e:
						print ("Service call failed: %s"%e)
					'''
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
	finally:
		f.close()


if __name__ == '__main__':
	# Initialize the node and name it.
	rospy.init_node('mission', anonymous = True)
	try:
		parser()
	except rospy.ROSInterruptException:
		print ("Mission Node Error")
		pass

