#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan
from math import radians
import numpy as np

class BehaviorSwitch(object):
    def __init__(self):
        self.running = False

    def callback(self, joy_msg):
        if joy_msg.buttons[0] == 1:
            self.running = not self.running

        rospy.loginfo(repr(joy_msg))

    def run(self):
        rospy.Subscriber('joy', Joy, self.callback)
        while (not rospy.is_shutdown()) and (not self.running):
			print("waiting for joy stick...")
			if self.running:
			    empty_msg = Twist()
			    cmd_vel_pub.publish(empty_msg)
			rate.sleep()

def scan_callback(scan):
	global g_range_ahead
	depths = []
	for dist in scan.ranges:
		if not np.isnan(dist):
			depths.append(dist)

	if len(depths) == 0:
		g_range_ahead = stop_thre
	else:
		g_range_ahead = min(depths)

	#g_range_ahead = min(depths)
	#g_range_ahead = msg.ranges[len(msg.ranges)/2]
	#print "range ahead: %0.2f" % g_range_ahead

sub = '/scan'
pub = '/cmd_vel_mux/input/teleop'
	
g_range_ahead = 1 # anything to start
scan_sub = rospy.Subscriber(sub, LaserScan, scan_callback)
cmd_vel_pub = rospy.Publisher(pub, Twist, queue_size=1) #'cmd_vel/velocityramp'
rospy.init_node('wander')
state_change_time = rospy.Time.now()
driving_forward = True
rate = rospy.Rate(10)
move = False
stop_thre = rospy.get_param("stop_threshold")


joy_run = BehaviorSwitch()
joy_run.run()

steer = np.random.uniform(-1,1)
while (not rospy.is_shutdown()) and joy_run.running:
	if driving_forward:
		if (g_range_ahead < stop_thre or rospy.Time.now() > state_change_time):
				driving_forward = False
				state_change_time = rospy.Time.now() + rospy.Duration(1)
	else: # we're not driving_forward
		if rospy.Time.now() > state_change_time:
				driving_forward = True # we're done spinning, time to go forward!
				state_change_time = rospy.Time.now() + rospy.Duration(30)
	twist = Twist()
	if driving_forward:
		twist.linear.x = 0.9
		twist.angular.z = steer
		#twist.linear.y = steer
	else:
		twist.angular.z = 0.9 #radians(45) #45deg/s in rad/s
		if steer != 0:
			steer = np.random.normal(0.5,0.2)

	cmd_vel_pub.publish(twist)

	rate.sleep()