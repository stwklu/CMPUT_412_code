#!/usr/bin/env python
from math import copysign

import rospy, cv2, cv_bridge, numpy
from tf.transformations import decompose_matrix, euler_from_quaternion
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ar_track_alvar_msgs.msg import AlvarMarkers
from ros_numpy import numpify
import numpy as np
from sensor_msgs.msg import Joy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import smach
import smach_ros
import actionlib
import tf

twist_pub = rospy.Publisher("teleop_velocity_smoother/raw_cmd_vel", Twist, queue_size=1)

client = None

# Global variables
# current_pose = None
linear_velocity = 0.3
rotate_velocity = 0.2
# search_origin = [(0, 0, 0), (0, 0, 0, 0)]
# y_threshold = 0.05
# x_threshold = 0.05
# y_scale = 1.0
# x_scale = 0.5
# max_angular_speed = 2.0
# min_angular_speed = 0.5
# max_linear_speed = 0.3
# min_linear_speed = 0.1
# goal_x = 0.6

# long_goal = None

current_id = 2

back_goal = None

current_time = rospy.Time.now()
rospy.Duration(24 * 60 * 60)

push_time = 10


# Search for the ar tag
class search(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['running', 'stop'])

    def execute(self, userdata):
        global back_goal
        target_pose = get_goal_pose()

        if target_pose is None:
            return 'running'
        else:
            back_goal = get_back_pose()
            client.send_goal(target_pose)
            client.wait_for_result()

            return 'stop'


# find the pose of the forward facing tag, then send goal
class move_close(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop'])

    def execute(self, userdata):
        client.send_goal(back_goal)
        client.wait_for_result()

        return 'stop'


# refine the pose using the backward facing tag
class move_from_back(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop'])

    def execute(self, userdata):
        global current_time
        target_pose = get_goal_pose()
        client.send_goal(target_pose)
        client.wait_for_result()

        current_time = rospy.Time.now()

        return 'stop'


# push
class push(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['running', 'finished'])

    def execute(self, userdata):
        if current_time + rospy.Duration(push) > rospy.Time.now():
            twist = Twist()
            twist.linear.x = linear_velocity
            twist.angular.z = rotate_velocity

            twist_pub.publish(twist)

            return 'running'

        else:
            twist = Twist()

            twist_pub.publish(twist)

            return 'finished'


def ar_callback(msg):
    global found, twist_pub, long_goal
    # try:
    #     marker = msg.markers[0]
    #     if not found:
    #         rospy.loginfo("Target Found...")
    #     found = True
    # except:
    #     if found:
    #         rospy.loginfo("Lost Target...")
    #     found = False
    #     long_goal = None
    #     return
    #
    # target_offset_y = marker.pose.pose.position.y
    # target_offset_x = marker.pose.pose.position.x
    #
    # twist = Twist()
    # if abs(target_offset_y) > y_threshold:
    #     speed = target_offset_y * y_scale
    #     twist.angular.z = copysign(max(min_angular_speed, min(max_angular_speed, abs(speed))), speed)
    # else:
    #     twist.angular.z = 0
    #
    # if abs(target_offset_x - goal_x) > x_threshold:
    #     speed = (target_offset_x - goal_x) * x_scale
    #     if speed < 0:
    #         speed *= 1.5
    #     twist.linear.x = copysign(min(max_linear_speed, max(min_linear_speed, abs(speed))), speed)
    # else:
    #     twist.linear.x = 0.0
    #     long_goal = None
    #     return
    #
    # long_goal = twist
    return


def odom_callback(msg):
    pose_msg = msg.pose.pose
    pose = numpify(pose_msg)

    __, __, angles, translate, __ = decompose_matrix(pose)
    print("Heading: ", angles[2] * 180 / 3.14159)


def get_goal_pose():
    listener = tf.TransformListener()

    mark_frame_id = '/item_' + str(current_id)
    try:
        listener.waitForTransform('/odom', mark_frame_id, rospy.Time(), rospy.Duration(4.0))
        (trans, rot) = listener.lookupTransform('/odom', mark_frame_id, rospy.Time(0))
    except:

        return None

    euler = tf.transformations.euler_from_quaternion(rot)
    #
    # euler[0] = 0.0
    # euler[1] = 0.0
    # euler[2] = (euler[2] + 3.1415926535 * 2) % (3.1415926535 * 2) - 3.1415926535

    # e3 = 0

    if euler[2] > 0:
        e3 = euler[2] - 3.1415926535
    else:
        e3 = euler[2] + 3.1415926535

    new_rot = tf.transformations.quaternion_from_euler(0.0, 0.0, euler[2])

    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = '/odom'
    goal_pose.target_pose.pose.position.x = trans[0]
    goal_pose.target_pose.pose.position.y = trans[1]
    goal_pose.target_pose.pose.position.z = 0.0
    goal_pose.target_pose.pose.orientation.x = 0.0
    goal_pose.target_pose.pose.orientation.y = 0.0
    goal_pose.target_pose.pose.orientation.z = new_rot[2]
    goal_pose.target_pose.pose.orientation.w = new_rot[3]

    return goal_pose


def get_back_pose():
    listener = tf.TransformListener()

    mark_frame_id = '/back_' + str(current_id)
    try:
        listener.waitForTransform('/odom', mark_frame_id, rospy.Time(), rospy.Duration(4.0))
        (trans, rot) = listener.lookupTransform('/odom', mark_frame_id, rospy.Time(0))
    except:

        return None

    euler = tf.transformations.euler_from_quaternion(rot)
    #
    # euler[0] = 0.0
    # euler[1] = 0.0
    # euler[2] = (euler[2] + 3.1415926535 * 2) % (3.1415926535 * 2) - 3.1415926535

    # e3 = 0

    if euler[2] > 0:
        e3 = euler[2] - 3.1415926535
    else:
        e3 = euler[2] + 3.1415926535

    new_rot = tf.transformations.quaternion_from_euler(0.0, 0.0, e3)

    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = '/odom'
    goal_pose.target_pose.pose.position.x = trans[0]
    goal_pose.target_pose.pose.position.y = trans[1]
    goal_pose.target_pose.pose.position.z = 0.0
    goal_pose.target_pose.pose.orientation.x = 0.0
    goal_pose.target_pose.pose.orientation.y = 0.0
    goal_pose.target_pose.pose.orientation.z = new_rot[2]
    goal_pose.target_pose.pose.orientation.w = new_rot[3]

    return goal_pose


def main():
    global cur_pos, twist_pub, client

    rospy.init_node('demo6')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['finished'])
    sm.userdata.sm_counter = 0

    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, ar_callback)
    rospy.Subscriber("/odom", Odometry, odom_callback)

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')

    sis.start()

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('search', search(),
                               transitions={'running': 'search',
                                            'stop': 'move_close'})
        smach.StateMachine.add('move_close', move_close(),
                               transitions={'stop': 'move_from_back'})
        smach.StateMachine.add('move_from_back', move_from_back(),
                               transitions={'stop': 'push'})
        smach.StateMachine.add('push', push(),
                               transitions={'running': 'push',
                                            'stop': 'finished'})

    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()
    sis.stop()
