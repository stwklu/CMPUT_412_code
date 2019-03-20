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
import tf

import numpy as np

# waypoints = [
#     [(3.24835373927, -1.40084858646, 0.0), (0.0, 0.0, -0.804220067176, 0.594331627588)],  # top left
#  8   [(1.74468228844, -2.84314082869, 0.0), (0.0, 0.0, 0.989949746297, 0.141419587774)],  # top right
#     [(0.945313977309, -1.26111663468, 0.0), (0.0, 0.0, 0.854504163474, 0.519444544302)],  # bottom left
#     [(-0.0848826540415, -2.39410610964, 0.0), (0.0, 0.0, -0.798635510047, 0.601815023152)]  # bottom right
# ]

waypoints = [

    [(3.32326580513, -1.35087213115, 0.0), (0.0, 0.0, 0.320751698119, 0.947163316516)],  # top right
    [(3.23505008652, -3.10500872923, 0.0), (0.0, 0.0, -0.0731199486702, 0.9973231538)],  # bottom right
    [(1.89342447299, -0.068353968551, 0.0), (0.0, 0.0, -0.041541443001, 0.999136253517)],  # top left
    # [(3.32326580513, -1.31087213115, 0.0), (0.0, 0.0, 0.320751698119, 0.947163316516)],  # top right

    [(1.97922349612, -2.24567663026, 0.0), (0.0, 0.0, -0.461773701893, 0.886997771878)],  # bottom left
    # [(3.23505008652, -3.06500872923, 0.0), (0.0, 0.0, -0.0731199486702, 0.9973231538)]  # bottom right

]

order_list = []

is_moving = False

linear_velocity = 0.15
rotate_velocity = 0.5

pose_msg = None


def goal_pose(pose):
    listener = tf.TransformListener()

    listener.waitForTransform('/odom', '/ar_marker_3', rospy.Time(), rospy.Duration(4.0))
    (trans, rot) = listener.lookupTransform('/odom', '/ar_marker_3', rospy.Time(0))

    # goal_pose = MoveBaseGoal()
    # goal_pose.target_pose.header.frame_id = '/map'
    # goal_pose.target_pose.pose.position.x = pose[0][0]
    # goal_pose.target_pose.pose.position.y = pose[0][1]
    # goal_pose.target_pose.pose.position.z = pose[0][2]
    # goal_pose.target_pose.pose.orientation.x = pose[1][0]
    # goal_pose.target_pose.pose.orientation.y = pose[1][1]
    # goal_pose.target_pose.pose.orientation.z = pose[1][2]
    # goal_pose.target_pose.pose.orientation.w = pose[1][3]

    print trans, rot

    return goal_pose


def joy_callback(msg):
    global order_list, is_moving

    print order_list, is_moving
    if (msg.buttons[0] == 1) and (0 not in order_list):
        order_list.append(0)
    elif (msg.buttons[1] == 1) and (1 not in order_list):
        order_list.append(1)
    elif (msg.buttons[2] == 1) and (2 not in order_list):
        order_list.append(2)
    elif (msg.buttons[3] == 1) and (3 not in order_list):
        order_list.append(3)
    else:
        pass

    if msg.axes[7] == 1.0:
        is_moving = True

    print order_list


def ar_pose_marker_callback(msg):
    global pose_msg

    if len(msg.markers) == 0:
        return

    pose_msg = msg.markers[0].pose.pose

    pose = numpify(pose_msg)
    __, __, angles, translate, __ = decompose_matrix(pose)

    cur_pos = translate[0:2]
    cur_heading = angles[2]

    # print "ar: " + str(angles)


if __name__ == '__main__':
    rospy.init_node('naving_square')
    rospy.Subscriber("/joy", Joy, joy_callback)
    # odom_sub = rospy.Subscriber('odom', Odometry, odom_callback)

    ar_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, ar_pose_marker_callback)

    # client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    # client.wait_for_server()
    while not rospy.is_shutdown():
        if pose_msg is not None:
            print "hh"
            goal = goal_pose(pose_msg)
