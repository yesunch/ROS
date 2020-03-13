#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import sys
import cv2
import message_filters
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

"""
Hints:
    Aruco markers:
        https://mecaruco2.readthedocs.io/en/latest/notebooks_rst/Aruco/Aruco.html
    Histogram Backprojection
        https://docs.opencv.org/3.4/dc/df6/tutorial_py_histogram_backprojection.html
    Meanshift & Camshift:
        https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_video/py_meanshift/py_meanshift.html
"""

class Node:

    def __init__(self, type_):
        """
        Initialization of aproximated synchronized subscription to topics.
        """
        self.mode = type_

        rospy.init_node('track_n_move')
        # Topic subscription
        rgb_sub = message_filters.Subscriber('/camera/rgb/image_raw', Image)
        depth_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)

        # Synchronization of received messages
        ats = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=5, slop=0.1)
        ats.registerCallback(self.callback)
        # ts = message_filters.TimeSynchronizer([rgb_sub, depth_sub], 1)
        # ts.registerCallback(self.callback)

        # OpenCV bridge to convert ROS image into OpenCV image
        self.bridge = CvBridge()

        # Publisher of a velocity commands        
        self.pub_vel = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)
        # Publisher of a processed image
        self.image_pub = rospy.Publisher('/processed_image', Image, queue_size=1)
        
        # Creating aruco_dict with 5x5 bits with max 250 ids
        # Ids ranges from 0-249
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
        
        # rate
        self.rate = rospy.Rate(1)
        
        # first circle
        self.firstCircle = True
        self.x1 = []
        
        # window dimension for meanshift
        self.x, self.y, self.w, self.h = 250, 120, 110, 160
        self.track_window = (self.x, self.y, self.w, self.h)
        self.firstImgForJoker = True
        
        # Velocity control message
        self.msg_vel = Twist()
        rospy.spin()

    def detect_aruco(self, img):
        """
        Wrapper around  OpenCV detectMarkers function
        Nothing to do here.
        :param img: cv image
        :return: list(dict(id: list(corners),...))
        """
        aruco_list = {}
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # lists of ids and the corners beloning to each id
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        if len(corners):
            for k in range(len(corners)):
                temp_1 = corners[k]
                temp_1 = temp_1[0]
                temp_2 = ids[k]
                temp_2 = temp_2[0]
                aruco_list[temp_2] = temp_1
            return aruco_list

    def mark_Aruco(self, img, aruco_list):
        """
        Nothing to do here.
        :param img: opencv image
        :param aruco_list: list of detected aruco markers
        :return img: opencv image with drawn marker centers
        :return centers: centers of aruco markers
        """
        key_list = aruco_list.keys()
        centers = []
        font = cv2.FONT_HERSHEY_SIMPLEX
        for key in key_list:
            dict_entry = aruco_list[key]    
            centre = dict_entry[0] + dict_entry[1] + dict_entry[2] + dict_entry[3]
            centre[:] = [int(x / 4) for x in centre] 
            orient_centre = centre + [0.0,5.0]
            centre = tuple(centre)  
            orient_centre = tuple((dict_entry[0]+dict_entry[1])/2)
            centers.append(list(centre))
            cv2.circle(img,centre,1,(0,0,255),8)
            cv2.line(img,centre,orient_centre,(255,0,0),4) 
        return img, centers

    def callback(self, rgb, depth):
        """
        This transforms a ROS image into an OpenCV image, 
        calls for a corresponding processing method and publishes a processed ROS image.
        Nothing to do here.
        :param data: ROS RGB and depth images
        """
        try:
            cv_rgb_image = self.bridge.imgmsg_to_cv2(rgb, "bgr8")
            cv_depth_image = self.bridge.imgmsg_to_cv2(depth, "32FC1")
        except CvBridgeError as e:
            rospy.loginfo('ROS img -> cv2 img conversion failure')
            return None

        if self.mode == "circle":
            # Part I: red circle
            cv_image = self.process_image_circle(cv_rgb_image, cv_depth_image)
        elif self.mode == "joker":
            # Part II: a random image
            cv_image = self.process_random_image(cv_rgb_image, cv_depth_image)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            rospy.loginfo('cv2 img -> ROS img conversion failure')

    def process_random_image(self, img, depth):
        """
        Processing of image with a random picture
        Hint 1: maybe, you will need to add a source of light in Gazebo.
                It is on the top panel.
        Hint 2: probably, it is worth to move the robot somehow. 
        :param img: opencv image
        :return img: processed opencv image
        """
        try:
            # Detect Aruco markers
            aruco_list = self.detect_aruco(img)
            # Draw their centers and find their centers
            img, centers = self.mark_Aruco(img, aruco_list)
        except Exception as e:
            print("No Aruco markers in the field of view.\n")
            return img

        # Your code starts here
        #x, y, w, h = 250, 120, 110, 160
        track_window = (self.x, self.y, self.w, self.h)
        if (self.firstImgForJoker is True):
        	roi = img[self.y:self.y+self.h, self.x:self.x+self.w]
        	hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        	mask = cv2.inRange(hsv_roi, np.array((50., 100., 100.)), np.array((180., 255., 255.)))
        	self.roi_hist = cv2.calcHist([hsv_roi], [0], mask, [180], [0,180])
        	cv2.normalize(self.roi_hist, self.roi_hist, 0, 255, cv2.NORM_MINMAX)
        	self.term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)
        	self.firstImgForJoker = False
        else:
        	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        	dst = cv2.calcBackProject([hsv], [0], self.roi_hist, [0,180],1)
        	ret, self.track_window = cv2.meanShift(dst, self.track_window, self.term_crit)
        	self.x, self.y, self.w, self.h = self.track_window
        	cv2.rectangle(img, (self.x, self.y), (self.x+self.w, self.y+self.h), 255, 2)
		    

        # Publish commands
        linear = 0.
        angular = 0.
        self.send_commands(linear, angular)        
        return img

    def process_image_circle(self, img, depth):
        """
        Processing of an image with a circle.
        :param img: opencv image
        :return img: processed opencv image
        """
        
        # Circle detection
        cimg = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)	# convert the image to HSV
        # define range of red color in HSV
        lower_red = np.array([0,100,100])
        upper_red = np.array([10,255,255])
        # Threshold the HSV image to get only red colors
        mask = cv2.inRange(cimg, lower_red, upper_red)
        # Bitwise-AND mask and original image
        cimg = cv2.bitwise_and(cimg,cimg, mask= mask)
        
        #try to find the circle
        grayImg = cv2.cvtColor(cimg, cv2.COLOR_BGR2GRAY)		 # get the gray image for the HoughCircles function
        circles = cv2.HoughCircles(grayImg,cv2.HOUGH_GRADIENT,1, 30, param1=50,param2=30,minRadius=0,maxRadius=0)	# compute the circle
        
        if (circles is not None):				# if circle found
        	print(">> Circle found")
        	circles = np.int16(np.around(circles))
        	circle = [circles[0][0][0], circles[0][0][1], circles[0][0][2]]
        	cv2.circle(img, (circle[0], circle[1]), circle[2], (0, 225, 0), 2)		# draw the circle
        	cv2.circle(img, (circle[0], circle[1]), 2, (0, 0, 255), 3)				# draw the circle
        	
        	if (self.firstCircle is True):			# if the circle is the first to be seen
        		self.x1 = [circle[0], circle[1]]		# store its position
        		self.firstCircle = False
        		linear = 0.
        		angular = 0.
        	else:  									# for other circles
        		xprime = [circle[0], circle[1]]		# store the current circle position
        		delta = [xprime[0] - self.x1[0], xprime[1] - self.x1[1]]	# compute the difference of position with the previous circle
        		self.x1 = xprime							# set the previous circle as this new circle
        		linear = float(float(delta[1])/200)		# send the linear mvt according to the delta computed
        		angular = float(float(delta[0])/500)   	# send the angular mvt according to the delta computed
        		print(">> Moving the robot")     		
        		self.send_commands(linear, -angular)	# move the robot
        	    
        # Publish necessary commands
        self.rate.sleep()     
        return img

        
    def send_commands(self, linear, angular):
        """
        Nothing to do here.
        :param cmds: dictionary of velocity commands
        """
        self.msg_vel.linear.x = linear
        self.msg_vel.angular.z = angular
        self.pub_vel.publish(self.msg_vel)


if __name__ == '__main__':
    if len(sys.argv) == 1:
        print("Looking for a circle")
        Node("circle")
    else:
        print("Looking for a {}".format(sys.argv[1]))
        Node(sys.argv[1])

