import sys
import copy
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

depthScale = 0
fx = 0
fy = 0

def pc2_to_xyzrgb(point):
    # Thanks to Panos for his code used in this function.
    x, y, z = point[:3]
    rgb = point[3]

    # cast float32 to int so that bitwise operations are possible
    s = struct.pack('>f', rgb)
    i = struct.unpack('>l', s)[0]
    # you can get back the float value by the inverse operations
    pack = ctypes.c_uint32(i).value
    r = (pack & 0x00FF0000) >> 16
    g = (pack & 0x0000FF00) >> 8
    b = (pack & 0x000000FF)
    return x, y, z, r, g, b


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

	y = y + 54
	z = z - 22
	x = x

	return x, y, z



def convert_x_y_value(x, y, z, w, h):

	# z is in cm, focal is constant of conversion
	centerY = y - w/2
	print (h)
	centerX = x - h/2

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

	return xw, yw

def device_orientation_wheel(image, depth_image):

    hsv_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

    lower_blue = (100,100,100)
    upper_blue = (130,255,255)


    mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

    result = cv2.bitwise_and(hsv_image, hsv_image, mask=mask)

    plt.subplot(1, 2, 1)
    plt.imshow(mask, cmap="gray")
    plt.subplot(1, 2, 2)
    plt.imshow(result)
    plt.show()

    # convert image to grayscale image
    rgb_image = cv2.cvtColor(result, cv2.COLOR_HSV2RGB)
    gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)

    # convert the grayscale image to binary image
    ret,thresh = cv2.threshold(gray_image,1,255,0)

    plt.imshow(thresh)
    plt.show()

    # calculate moments of binary image
    M = cv2.moments(thresh)
     
    # calculate x,y coordinate of center
    cX1 = int(M["m10"] / M["m00"])
    cY1 = int(M["m01"] / M["m00"])


    image[:, 0:cY1 - 70] =  0
    image[:, cY1 + 70:] =  0
    hsv_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

    #get white part centroid

    light_white = (0, 0, 160)
    dark_white = (100, 60, 255) 

    lo_square = np.full((10, 10, 3), light_white, dtype=np.uint8) / 255.0
    do_square = np.full((10, 10, 3), dark_white, dtype=np.uint8) / 255.0

    plt.subplot(1, 2, 1)
    plt.imshow(hsv_to_rgb(do_square))
    plt.subplot(1, 2, 2)
    plt.imshow(hsv_to_rgb(lo_square))
    plt.show()

    mask = cv2.inRange(hsv_image, light_white, dark_white)

    result = cv2.bitwise_and(hsv_image, hsv_image, mask=mask)

    plt.subplot(1, 2, 1)
    plt.imshow(mask, cmap="gray")
    plt.subplot(1, 2, 2)
    plt.imshow(result)
    plt.show()

    rgb_image = cv2.cvtColor(result, cv2.COLOR_HSV2RGB)
    gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)

    ret,thresh = cv2.threshold(gray_image,1,255,0)

    plt.imshow(thresh)
    plt.show()


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

    plt.subplot(1, 2, 1)
    plt.imshow(hsv_to_rgb(do_square))
    plt.subplot(1, 2, 2)
    plt.imshow(hsv_to_rgb(lo_square))
    plt.show()

    mask = cv2.inRange(hsv_image, lower_green, upper_green)

    result = cv2.bitwise_and(hsv_image, hsv_image, mask=mask)

    plt.subplot(1, 2, 1)
    plt.imshow(mask, cmap="gray")
    plt.subplot(1, 2, 2)
    plt.imshow(result)
    plt.show()

    rgb_image = cv2.cvtColor(result, cv2.COLOR_HSV2RGB)
    gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)

    ret,thresh = cv2.threshold(gray_image,1,255,0)

    plt.imshow(thresh)
    plt.show()
     
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

    dist = math.hypot(cX2 - cX1, cY2 - cY1)
    orientation = 1
    if (dist > 80):
    	orientation = 2



    return x, y, depth, orientation




def device_orientation_shuttle(image):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

    lower_blue = (100,130,100)
    upper_blue = (130,255,255)

    lo_square = np.full((10, 10, 3), lower_blue, dtype=np.uint8) / 255.0
    do_square = np.full((10, 10, 3), upper_blue, dtype=np.uint8) / 255.0

    plt.subplot(1, 2, 1)
    plt.imshow(hsv_to_rgb(do_square))
    plt.subplot(1, 2, 2)
    plt.imshow(hsv_to_rgb(lo_square))
    plt.show()

    mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

    result = cv2.bitwise_and(hsv_image, hsv_image, mask=mask)

    plt.subplot(1, 2, 1)
    plt.imshow(mask, cmap="gray")
    plt.subplot(1, 2, 2)
    plt.imshow(result)
    plt.show()

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


    light_white = (0, 0, 160)
    dark_white = (100, 60, 255)

    lo_square = np.full((10, 10, 3), light_white, dtype=np.uint8) / 255.0
    do_square = np.full((10, 10, 3), dark_white, dtype=np.uint8) / 255.0

    plt.subplot(1, 2, 1)
    plt.imshow(hsv_to_rgb(do_square))
    plt.subplot(1, 2, 2)
    plt.imshow(hsv_to_rgb(lo_square))
    plt.show()

    mask = cv2.inRange(hsv_image, light_white, dark_white)

    result = cv2.bitwise_and(hsv_image, hsv_image, mask=mask)

    plt.subplot(1, 2, 1)
    plt.imshow(mask, cmap="gray")
    plt.subplot(1, 2, 2)
    plt.imshow(result)
    plt.show()

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

    orientation = 1
    if (abs(pipes[0][1] - pipes[0][1]) <= 20):
    	orientation = 2


    return x, y, depth, orientation







def device_orientation_breaker(image, switchNum):

    light_orange = (1, 120, 125)
    dark_orange = (17, 255, 255)

    hsv_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)


    mask = cv2.inRange(hsv_image, light_orange, dark_orange)

    result = cv2.bitwise_and(hsv_image, hsv_image, mask=mask)

    plt.subplot(1, 2, 1)
    plt.imshow(mask, cmap="gray")
    plt.subplot(1, 2, 2)
    plt.imshow(result)
    plt.show()

    # convert image to grayscale image
    rgb_image = cv2.cvtColor(result, cv2.COLOR_HSV2RGB)
    gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)

    # convert the grayscale image to binary image
    ret,thresh = cv2.threshold(gray_image,1,255,0)

    plt.imshow(thresh)
    plt.show()

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


    switchn = int(switchNum[1]) - 1
    
    position = switches[switchn]

    cX1, cY1 = position

    _, _, depth = find_closest_area(depth_image[cX1- 50: cX1 + 50, cY1 - 50:cY1 + 50])

    depth = depth * depthScale * 100

    print (depth)

    w, h = depth_image.shape

    x, y = convert_x_y_value(cX1, cY1, depth, w, h)



    return x, y, depth




if __name__ == '__main__':

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


	print (w)
	print (h)
	print (depth_intrinsics)
	

	clipping_distance_in_meters = 1.1 #1 meter
	inner_clipping_distance_in_meters = 0.5
	clipping_distance = clipping_distance_in_meters / depth_scale
	inner_clipping_distance = inner_clipping_distance_in_meters / depth_scale
	
	align_to = rs.stream.color
	align = rs.align(align_to)


	filename = sys.argv[-1]

	f = open(filename,'r')
	lines = f.readlines()

	for line in lines:
		lineSplit = line.split(", ")
		stations = [x[0] for x in lineSplit[:-1]]

		times = lineSplit[-1]
		lineSplit = [x[1:] for x in lineSplit[:-1]]

		print ("Time for Task " + times)

		for i in range(0, len(lineSplit)):
			try:
				print('In Try!')
				input("Press Enter to Get Frame...")
				print('Detecting Object!')

				i = 4

				
				frameset = pipe.wait_for_frames()
				print('Got Frameset!')

				aligned_frames = align.process(frameset)

				color_frame = aligned_frames.get_color_frame()
				depth_frame = aligned_frames.get_depth_frame()


				print(depth_frame)

				depth_image = np.asanyarray(depth_frame.get_data())
				color_image = np.asanyarray(color_frame.get_data())


				depth_image = depth_image[300:, 400:880]
				color_image = color_image[300:, 400:880]
				


				depth_image = np.load("shuttle5_depth.npy")
				color_image = np.load("shuttle5_color.npy")
				#depth_image.tofile("wheelvalve3_depth.txt")
				#color_image.tofile("wheelvalve3_color.txt")

				#np.save("shuttle5_depth.npy", depth_image)
				#np.save("shuttle5_color.npy", color_image)
   
				print (depth_image.shape)
				print (color_image.shape)

				grey_color = 0
				depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels

				color_image_bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d < inner_clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

				plt.subplot(1, 3, 1)
				plt.imshow(color_image)
				plt.subplot(1, 3, 2)
				plt.imshow(depth_image)
				plt.subplot(1, 3, 3)
				plt.imshow(color_image_bg_removed)
				plt.show()

				print (depth_image)

				w, h = depth_image.shape
				fdepth_image = median_filter(depth_image)

				x, y, z = find_closest_area(fdepth_image)
				x1, y2, z3 = find_closest_area(depth_image)



				task = lineSplit[i]
				tasks = task.split(" ")
				#We know it is a Valve
				if (len(tasks) == 2):
					printStatement = "Go to Station " + stations[i]

					types = tasks[0]
					orientation = int(tasks[1])
					if (types == "V3"):
						x, y, z, o = device_orientation_shuttle(color_image_bg_removed)
						x, y, z = translate_from_camera_to_arm_frame(x, y, z)
						if orientation == 1:
							print (printStatement + " and close shuttlecock valve")
						else:
							print (printStatement + " and open shuttlecock valve")

						print ("Device relative to Arm frame")
						print ("(" + str(x) + ", " + str(y)  + ", " + str(z)  + ", " + str(o) + ")" )

					else:
						x, y, z, o = device_orientation_wheel(color_image_bg_removed, depth_image)
						x, y, z = translate_from_camera_to_arm_frame(x, y, z)
						print (printStatement + " and rotate wheel valve to " + str(orientation) + " degrees")
						print ("Device relative to Arm frame")
						print ("(" + str(x) + ", " + str(y)  + ", " + str(z)  + ", " + str(o) + ")" )


				else:
					printStatement = "Go to Station " + stations[i]
					printStatement = printStatement + " For Breaker " + tasks[0]
					for j in range(1, len(tasks), 2):

						switch = tasks[j]
						orientation = tasks[j + 1]
						x, y, z = device_orientation_breaker(color_image_bg_removed, switch)
						if (orientation == "U"):
							printStatement = printStatement + " flip switch " + switch[1:] + " up"


						else:
							printStatement = printStatement + " flip switch " + switch[1:] + " down"


						print (printStatement)
						print ("Device relative to Arm frame")
						x, y, z = translate_from_camera_to_arm_frame(x, y, z)
						print ("(" + str(x) + ", " + str(y)  + ", " + str(z)  +  ")" )



			except Exception as e: 
				print(e)
				print('In EXCEPT!')
				break

		print ("Processing .....")
		time.sleep(22)


	#pipe.stop()
	f.close()