#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers
from nav_msgs.msg import Odometry
from ros_numpy import numpify
from tf.transformations import decompose_matrix, euler_from_quaternion

import numpy as np


def odom_callback(msg):
    # global cur_pos, cur_heading

    pose_msg = msg.pose.pose

    pose = numpify(pose_msg)
    __, __, angles, translate, __ = decompose_matrix(pose)

    cur_pos = translate[0:2]
    cur_heading = angles[2]

    # print "odom: " + str(cur_heading)


def ar_pose_marker_callback(msg):
    if len(msg.markers) == 0:
        return

    pose_msg = msg.markers[0].pose.pose

    pose = numpify(pose_msg)
    __, __, angles, translate, __ = decompose_matrix(pose)

    cur_pos = translate[0:2]
    cur_heading = angles[2]

    print "ar: " + str(angles)
    # print msg.markers[0].pose
    # print "______________________________________"


rospy.init_node('range_ahead')

odom_sub = rospy.Subscriber('odom', Odometry, odom_callback)

ar_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, ar_pose_marker_callback)

rospy.spin()
