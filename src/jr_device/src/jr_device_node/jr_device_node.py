#!/usr/bin/env python

import sys
import copy
import rospy
import time
import numpy as np
import matplotlib
import math
matplotlib.use('TkAgg')
from matplotlib import pyplot as plt
import skimage.io
import pyrealsense2 as rs
import cv2
import matplotlib.pyplot as plt
from matplotlib.colors import hsv_to_rgb
from scipy import ndimage
import keyboard
from PIL import Image
from jr_actuation.srv import *
from jr_kinematics.srv import *
from jr_device.srv import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist


depthScale = 0
fx = 0
fy = 0
align = None
pipe = None
clipping_distance = 1
inner_clipping_distance = 1
special_station_offset = 0
depth_image = None

def median_filter(img):

    members = [0] * 9
    (width, height) = img.shape
    newimg = np.empty([width,height], dtype=int)
    for i in range(1,width-1):
        for j in range(1,height-1):
            members[0] = img[i-1, j-1]         
            members[1] = img[i, j-1] 
            members[2] = img[i+1, j-1] 
            members[3] = img[i-1, j] 
            members[4] = img[i, j] 
            members[5] = img[i+1, j] 
            members[6] = img[i-1, j+1] 
            members[7] = img[i, j+1] 
            members[8] = img[i+1, j+1] 
            members.sort()
            newimg[i,j] = (members[4])
    return newimg

def find_closest_area(img):

    distance = 100000
    x = 0
    y = 0

    (width, height) = img.shape
    newimg = np.empty([width,height], dtype=int)
    for i in range(1,width-1):
        for j in range(1,height-1):
            dValue = img[i, j]   
            if (dValue > 0 and dValue < distance):
                d1 = img[i, j - 1]
                d2 = img[i - 1, j]
                d3 = img[i+ 1, j]
                d4 = img[i, j + 1]

                if ((d1 + d2 + d3 + d4)/4 <= (dValue + 10) and
                    (d1 + d2 + d3 + d4)/4 >= (dValue - 10)):
                    distance = dValue
                    x = i
                    y = j

    return x, y, distance


def translate_from_camera_to_arm_frame(x, y, z):

	y = y + 61
	z = z - 22
	x = x

	return x, y, z



def convert_x_y_value(x, y, z, w, h):

	# z is in cm, focal is constant of conversion
	centerY = y - w/2
	print (h)
	centerX = x - h/2 + special_station_offset

	print (centerY)
	print (centerX)


	yw = z * centerY / (fy)
	xw = z * centerX / (fx)

	yw = -yw

	print (xw)

	#offset from camera view
	yw = yw - 16
	print (yw)
	print (xw)

	return xw/2, yw

def device_orientation_wheel(image, depth_image):

	hsv_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

	lower_blue = (100,100,80)
	upper_blue = (130,255,255)


	mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

	result = cv2.bitwise_and(hsv_image, hsv_image, mask=mask)


	# convert image to grayscale image
	rgb_image = cv2.cvtColor(result, cv2.COLOR_HSV2RGB)
	gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)

	# convert the grayscale image to binary image
	ret,thresh = cv2.threshold(gray_image,1,255,0)


	# calculate moments of binary image
	M = cv2.moments(thresh)
	 
	# calculate x,y coordinate of center
	cX1 = int(M["m10"] / M["m00"])
	cY1 = int(M["m01"] / M["m00"])


	image[:, 0:cY1 - 70] =  0
	image[:, cY1 + 70:] =  0
	hsv_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

	#get white part centroid

	light_white = (0, 0, 140)
	dark_white = (100, 60, 255) 

	lo_square = np.full((10, 10, 3), light_white, dtype=np.uint8) / 255.0
	do_square = np.full((10, 10, 3), dark_white, dtype=np.uint8) / 255.0


	mask = cv2.inRange(hsv_image, light_white, dark_white)

	result = cv2.bitwise_and(hsv_image, hsv_image, mask=mask)


	rgb_image = cv2.cvtColor(result, cv2.COLOR_HSV2RGB)
	gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)

	ret,thresh = cv2.threshold(gray_image,1,255,0)



	M = cv2.moments(thresh)
	 
	# calculate x,y coordinate of center
	cX2 = int(M["m10"] / M["m00"])
	cY2 = int(M["m01"] / M["m00"])


	#look for green

	print ("Looking for Green")

	#modify hsv image
	rgb_image = cv2.cvtColor(result, cv2.COLOR_HSV2RGB)
	rgb_image = rgb_image[cX1 - 50: cX1 + 50, :]
	hsv_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)


	image[0:cX1 - 50, :] =  0
	image[cX1 + 50:, :] =  0
	hsv_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)



	lower_green = (40,50,40)
	upper_green = (70,255,255) # cv2.cvtColor(green,cv2.COLOR_BGR2HSV)


	lo_square = np.full((10, 10, 3), lower_green, dtype=np.uint8) / 255.0
	do_square = np.full((10, 10, 3), upper_green, dtype=np.uint8) / 255.0


	mask = cv2.inRange(hsv_image, lower_green, upper_green)

	result = cv2.bitwise_and(hsv_image, hsv_image, mask=mask)

	rgb_image = cv2.cvtColor(result, cv2.COLOR_HSV2RGB)
	gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)

	ret,thresh = cv2.threshold(gray_image,1,255,0)

	# calculate moments of binary image
	M = cv2.moments(thresh)
	 
	# calculate x,y coordinate of center
	#cX3 = int(M["m10"] / M["m00"])
	#cY3 = int(M["m01"] / M["m00"])


	print ("Centroids")

	print (cX1)
	print (cY1)
	print (cX2)
	print (cY2)
	#print (cX3)
	#print (cY3)


	#myradians = math.atan2(cX3-cX1, cY3-cY1)

	#mydegrees = math.degrees(myradians)

	print ("DEGREES")
	#print (mydegrees)


	depth = depth_image[cX1, cY1]
	print (depth)

	_, _, depth = find_closest_area(depth_image[cX1- 50: cX1 + 50, cY1 - 50:cY1 + 50])

	depth = depth * depthScale * 100

	print (depth)

	w, h = depth_image.shape

	x, y = convert_x_y_value(cX1, cY1, depth, w, h)

	dist = cY2 - cY1
	orientation = 0
	if (dist > 80):
		orientation = 1


	return x, y, depth, orientation




def device_orientation_shuttle(image):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

    lower_blue = (100,130,90)
    upper_blue = (130,255,255)

    lo_square = np.full((10, 10, 3), lower_blue, dtype=np.uint8) / 255.0
    do_square = np.full((10, 10, 3), upper_blue, dtype=np.uint8) / 255.0

    mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

    result = cv2.bitwise_and(hsv_image, hsv_image, mask=mask)


    rgb_image = cv2.cvtColor(result, cv2.COLOR_HSV2RGB)
    gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)

    ret,thresh = cv2.threshold(gray_image,1,255,0)

    print ("Finding Centroid")

    # calculate moments of binary image
    M = cv2.moments(thresh)
     
    # calculate x,y coordinate of center
    cX1 = int(M["m10"] / M["m00"])
    cY1 = int(M["m01"] / M["m00"])

    print (str(cX1) + " " + str(cY1))

    #contours, hierarchy = cv2.findContours(thresh,1, 2)
    #print ("FINDING COUNTOUR")
    #cnt = contours[0]
    #x,y,w,h = cv2.boundingRect(cnt)
    #print ("FOUND COUNTOUR")


    light_white = (0, 0, 140)
    dark_white = (100, 60, 255)

    lo_square = np.full((10, 10, 3), light_white, dtype=np.uint8) / 255.0
    do_square = np.full((10, 10, 3), dark_white, dtype=np.uint8) / 255.0

    mask = cv2.inRange(hsv_image, light_white, dark_white)

    result = cv2.bitwise_and(hsv_image, hsv_image, mask=mask)

    rgb_image = cv2.cvtColor(result, cv2.COLOR_HSV2RGB)
    gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)

    ret,thresh = cv2.threshold(gray_image,1,255,0)

    print ("Finding Countours")

    m2, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    pipes = []

    cntsSorted = sorted(contours, key=lambda x: cv2.contourArea(x))
    print (cntsSorted)
    cntsSorted = cntsSorted[-2:]
    for c in cntsSorted:
        # calculate moments for each contour
        M = cv2.moments(c)

        # calculate x,y coordinate of center
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])     
        pipes.append((cX, cY))

    print (pipes)

    _, _, depth = find_closest_area(depth_image[cX1- 50: cX1 + 50, cY1 - 50:cY1 + 50])

    depth = depth * depthScale * 100

    print (depth)

    w, h = depth_image.shape

    x, y = convert_x_y_value(cX1, cY1, depth, w, h)

    orientation = 0
    if ((abs(pipes[0][1] - pipes[1][1]) <= 20) and (abs(pipes[0][0] - pipes[1][0]) > 120)):
    	orientation = 1

    #closed
    pos = 1
    #open
    if (orientation == 1 and (max(pipes[0][1], pipes[1][1]) - y) > 30):
        pos = 0

    if (orientation == 0 and abs((pipes[0][0] + pipes[1][0])/2 - x) <= 30):
        pos = 0

    print ("Position")
    print (pos)


    return x, y, depth, orientation, pos




def device_orientation_breaker(image, switchNum):

    light_orange = (1, 120, 125)
    dark_orange = (17, 255, 255)

    hsv_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)


    mask = cv2.inRange(hsv_image, light_orange, dark_orange)

    result = cv2.bitwise_and(hsv_image, hsv_image, mask=mask)

    # convert image to grayscale image
    rgb_image = cv2.cvtColor(result, cv2.COLOR_HSV2RGB)
    gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)

    # convert the grayscale image to binary image
    ret,thresh = cv2.threshold(gray_image,1,255,0)

    # calculate moments of binary image
    im2, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    switches = []

    cntsSorted = sorted(contours, key=lambda x: cv2.contourArea(x))
    print (cntsSorted)
    cntsSorted = cntsSorted[-3:]
    for c in cntsSorted:
        # calculate moments for each contour
        M = cv2.moments(c)

        # calculate x,y coordinate of center
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])     
        switches.append((cX, cY))


    print (switches)

    switches.sort(key=lambda x: x[0])

    print (switches)

    position = switches[switchNum]

    cX1, cY1 = position

    _, _, depth = find_closest_area(depth_image[cX1- 50: cX1 + 50, cY1 - 50:cY1 + 50])

    depth = depth * depthScale * 100

    print (depth)

    w, h = depth_image.shape

    x, y = convert_x_y_value(cX1, cY1, depth, w, h)



    return x, y, depth



def run_pose_detection():
	global pipe
	global fx
	global fy
	global align
	global clipping_distance
	global inner_clipping_distance
	global depthScale
	# Declare pointcloud object, for calculating pointclouds and texture mappings
	pc = rs.pointcloud()
	# We want the points object to be persistent so we can display the last cloud when a frame drops
	points = rs.points()

	# Declare RealSense pipeline, encapsulating the actual device and sensors
	pipe = rs.pipeline()

	config = rs.config()
	config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)
	config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)


	#Start streaming with default recommended configuration
	
	try:
		profile = pipe.start(config)

		depth_sensor = profile.get_device().first_depth_sensor()
		depth_scale = depth_sensor.get_depth_scale()
		print("Depth Scale is: " , depth_scale)
		depthScale = depth_scale

		profile2 = pipe.get_active_profile()
		depth_profile = rs.video_stream_profile(profile2.get_stream(rs.stream.depth))
		depth_intrinsics = depth_profile.get_intrinsics()
		w, h = depth_intrinsics.width, depth_intrinsics.height

		fx, fy = depth_intrinsics.fx, depth_intrinsics.fy


		clipping_distance_in_meters = 1.1 #1 meter
		inner_clipping_distance_in_meters = 0.5
		clipping_distance = clipping_distance_in_meters / depth_scale
		inner_clipping_distance = inner_clipping_distance_in_meters / depth_scale
		
		align_to = rs.stream.color
		align = rs.align(align_to)


		filename = sys.argv[-1]

		f = open(filename,'r')
		lines = f.readlines()


		rospy.init_node('pose_detection_server')
		s = rospy.Service('pose_srv', PoseCmd, detect_pose)


		print "Ready for pose detection"
		rospy.spin()
	finally:
		pipe.stop()
		f.close()


def detect_pose(req):
	types = req.dev_type
	global depth_image
	global special_station_offset
	special_station_offset = 0

	try:
		frameset = pipe.wait_for_frames()
		print('Got Frameset!')

		aligned_frames = align.process(frameset)

		color_frame = aligned_frames.get_color_frame()
		depth_frame = aligned_frames.get_depth_frame()


		station = req.station_id

		print ('Station')
		print (station)

		print ('Types')
		print (types)

		depth_image = np.asanyarray(depth_frame.get_data())
		color_image = np.asanyarray(color_frame.get_data())

		if (station == 4):
			special_station_offset = 120
		if (station == 5):
			special_station_offset = -120

		depth_image = depth_image[300:, 440+special_station_offset:840+special_station_offset]
		color_image = color_image[300:, 440+special_station_offset:840+special_station_offset]
		

		print (depth_image.shape)
		print (color_image.shape)

		grey_color = 0
		depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels

		color_image_bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d < inner_clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

		printStatement = ""
		x = 0
		y = 0
		z = 0
		o = 0
		print ("Identifying Device")
		if (types == 2):
			x, y, z, o = device_orientation_shuttle(color_image_bg_removed)
			x, y, z = translate_from_camera_to_arm_frame(x, y, z)
			
			print ("Device relative to Arm frame")
			print ("(" + str(x) + ", " + str(y)  + ", " + str(z)  + ", " + str(o) + ")" )

		elif (types == 0 or types == 1):
			x, y, z, o = device_orientation_wheel(color_image_bg_removed, depth_image)
			x, y, z = translate_from_camera_to_arm_frame(x, y, z)
			print ("Device relative to Arm frame")
			print ("(" + str(x) + ", " + str(y)  + ", " + str(z)  + ", " + str(o) + ")" )


		elif (types == 3):
			printStatement = printStatement + " For Breaker "
			switch = req.dev_id
		
			x, y, z = device_orientation_breaker(color_image_bg_removed, switch)
	
			print (printStatement)
			print ("Device relative to Arm frame")
			x, y, z = translate_from_camera_to_arm_frame(x, y, z)
			print ("(" + str(x) + ", " + str(y)  + ", " + str(z)  +  ")" )

		#points = Point(float(x),float(y),float(z))
		points = Point(float(z),float(x),float(y + 10))
		quaternion = Quaternion(float(z),float(x),float(y + 10), float(o))
		resp = PoseCmdResponse()
		resp.pose = Pose(points, quaternion)
		resp.dev_ori = o
		resp.dev_pos = 1
		return resp

	except Exception as e: 
		print(e)
		print('In EXCEPT!')
		points = Point(0.0,0.0,0.0)
		quaternion = Quaternion(0.0,0.0,0.0,0.0)
		resp = PoseCmdResponse()
		resp.pose = Pose(points, quaternion)
		resp.dev_ori = 1
		return resp

	print ("Processing .....")


if __name__ == '__main__':
	run_pose_detection()
