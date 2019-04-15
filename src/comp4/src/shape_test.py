#!/usr/bin/env python
from math import copysign

import rospy, cv2, cv_bridge, numpy, math
from tf.transformations import decompose_matrix, euler_from_quaternion
from sensor_msgs.msg import Image, CameraInfo
from ar_track_alvar_msgs.msg import AlvarMarkers
from ros_numpy import numpify
import numpy as np
from std_msgs.msg import String
from tf import transformations, TransformerROS
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import smach
import smach_ros
import tf
import image_geometry

import traceback
import logging

bridge = CvBridge()
shape_x = None
shape_y = None

TRIANGLE = 1
CIRCLE = 2
RECTANGLE = 3
object_type = CIRCLE
current_type = 0

anguler_scale = -0.006
max_angular_speed = 0.6
min_angular_speed = 0.2
image_err_threshold = 0
image_err_back_threshold = 10

range_ahead = 10

def scan_callback(msg):
    global range_ahead
    range_ahead = msg.ranges[len(msg.ranges)/2]

def image_callback(msg):
	global shape_x, shape_y, current_type
	rospy.loginfo('Got image...')
	image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	lower_red = numpy.array([100,150,50])
	upper_red = numpy.array([255,255,255])
	red_mask = cv2.inRange(hsv, lower_red, upper_red)
	h, w = red_mask.shape
	upper_padding = 120
	lower_padding = 200
	red_mask[0:upper_padding, 0:w] = 0
	red_mask[h-lower_padding:w, 0:w] = 0
	image[0:upper_padding, 0:w] = 0
	image[h-lower_padding:w, 0:w] = 0

	# Default image error
	shape_x = None
	shape_y = None


	im2, red_contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	for item in red_contours:

	    if cv2.contourArea(item) > 1000:

	        peri = cv2.arcLength(item, True)
	        approx = cv2.approxPolyDP(item, 0.04 * peri, True)

	        if len(approx) == 3:
	            current_type = TRIANGLE
	        elif len(approx) == 4:
	            current_type = RECTANGLE
	        else:
	            current_type = CIRCLE

	        if current_type == object_type:
	            (shape_x, shape_y), shape_radius = cv2.minEnclosingCircle(item)
	            break


	twist = Twist()

	if shape_x == None:
		twist.angular.z = min_angular_speed
		twist_pub.publish(twist)
		cv2.imshow("image_shape", image)
		cv2.waitKey(3)
		return
	else:
		cv2.circle(image, (int(math.ceil(shape_x)), int(math.ceil(shape_y))), 20, (0, 0, 255), -1)
		cv2.imshow("image shape", image)
		cv2.waitKey(3)

	if range_ahead >= 0.7:
		twist.linear.x = 0.1
		if abs(shape_x- w/2) > image_err_threshold:
			anguler_speed = (shape_x- w/2) * anguler_scale
			twist.angular.z = math.copysign(max(min_angular_speed, min(max_angular_speed, abs(anguler_speed))), anguler_speed)
			print("image error: ", shape_x - w/2)
			print("anguler_speed", anguler_speed)
		twist_pub.publish(twist)
	else:
		print("stop............................")



rospy.init_node('part_1')
twist_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=1)
img_sub = rospy.Subscriber('camera/rgb/image_raw',Image, image_callback)
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
rospy.spin()