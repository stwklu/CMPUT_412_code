#!/usr/bin/env python
from math import copysign

import rospy, cv2, cv_bridge, numpy
from tf.transformations import decompose_matrix, euler_from_quaternion
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ros_numpy import numpify
import numpy as np
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers
import tf
from kobuki_msgs.msg import Sound
import smach
import smach_ros

# Global variables
twist_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=1)
current_pose = [(0, 0, 0), (0, 0, 0, 0)]
linear_velocity = 0.15
rotate_velocity = 0.5
current_id = 3

target_pose = None

search_origin = [(0, 0, 0), (0, 0, 0, 0)]
y_threshold = 0.05
x_threshold = 0.05
y_scale = 0.2
x_scale = 0.5
max_angular_speed = 2.0
min_angular_speed = 0.5
max_linear_speed = 0.3
min_linear_speed = 0.1
goal_x = 0.9

long_goal = None
found = False
is_moving = False

sound_pub = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=1)
# client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
# client.wait_for_server()

client = None
target_offset_y = None
target_offset_x = None

temp_twist = Twist()


class move_in(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['moving', 'arrived'])

    def execute(self, userdata):
        global found, long_goal
        if is_moving:

            return 'arrived'
        else:
            twist_pub.publish(temp_twist)
            found = False
            long_goal = None
            return 'moving'


class rotate_check(smach.State):
    global found, twist_pub

    def __init__(self):
        smach.State.__init__(self, outcomes=['found', 'not_found', 'finished'])

    def execute(self, userdata):

        if current_id < 4 and long_goal:
            twist = Twist()
            twist_pub.publish(twist)
            return 'found'
        elif current_id < 4 and (long_goal is None):

            if found:
                twist = Twist()
                twist_pub.publish(twist)

                return 'found'

            else:

                twist = Twist()
                twist.linear.x = 0
                twist.angular.z = rotate_velocity
                twist_pub.publish(twist)
                return 'not_found'
        else:
            return 'finished'


class rough_close(smach.State):
    global found, twist_pub

    def __init__(self):
        smach.State.__init__(self, outcomes=['moving', 'arrived'])

    def execute(self, userdata):

        if long_goal:

            twist_pub.publish(long_goal)
            return 'moving'
        else:
            return 'arrived'


class moving_towards(smach.State):
    global long_goal, twist_pub, found

    def __init__(self):
        smach.State.__init__(self, outcomes=['moving', 'arrived', 'rotating'])

    def execute(self, userdata):
        global target_pose, found

        target_pose = get_goal_pose()
        if target_pose is None:
            found = False
            return 'rotating'
        else:

            client.send_goal(target_pose)
            client.wait_for_result()
            target_pose = None
            return 'arrived'


class docking(smach.State):
    """docstring for docking"""

    def __init__(self):
        smach.State.__init__(self, outcomes=['finished'])

    def execute(self, userdata):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        twist_pub.publish(twist)
        rospy.sleep(3)
        return 'finished'


class go_back(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['moving'])

    def execute(self, userdata):
        global found, current_id, long_goal
        client.send_goal(search_origin)
        client.wait_for_result()
        found = False
        current_id += 1
        long_goal = None

        return 'moving'


def joy_callback(msg):
    global is_moving, temp_twist, search_origin
    if msg.axes[-1] == 1.0:
        temp_twist.linear.x = linear_velocity
        temp_twist.angular.z = 0

    elif msg.axes[-1] == -1.0:
        temp_twist.linear.x = -linear_velocity
        temp_twist.angular.z = 0
    elif msg.axes[-2] == 1.0:
        temp_twist.linear.x = 0
        temp_twist.angular.z = rotate_velocity
    elif msg.axes[-2] == -1.0:
        temp_twist.linear.x = 0
        temp_twist.angular.z = -rotate_velocity
    else:
        pass

    if msg.buttons[0] == 1:
        temp_twist.linear.x = 0
        temp_twist.angular.z = 0
    # memorize search origin
    if msg.buttons[1] == 1:
        search_origin = current_pose
        search_origin = get_initial_pose(search_origin)
        sound_pub.publish(0)

    if msg.buttons[2] == 1:
        is_moving = True


# def ar_callback(msg):
#     global target_pose
#     if len(msg.markers) == 0:
#         return
#
#     for marker in msg.markers:
#
#         marker_id = int(marker.id)
#
#         if marker_id == current_id:
#             target_pose = get_goal_pose(marker.pose.pose)
#             return

def get_goal_pose():
    listener = tf.TransformListener()

    mark_frame_id = '/ar_marker_' + str(current_id)
    try:
        listener.waitForTransform('/odom', mark_frame_id, rospy.Time(), rospy.Duration(4.0))
        (trans, rot) = listener.lookupTransform('/odom', mark_frame_id, rospy.Time(0))
    except:

        return None

    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = '/odom'
    goal_pose.target_pose.pose.position.x = 1.0
    goal_pose.target_pose.pose.position.y = 0.0
    goal_pose.target_pose.pose.position.z = 0.0
    goal_pose.target_pose.pose.orientation.x = 0.0
    goal_pose.target_pose.pose.orientation.y = 0.0
    goal_pose.target_pose.pose.orientation.z = 0.0
    goal_pose.target_pose.pose.orientation.w = 1.0

    return goal_pose


def ar_callback(msg):
    global found, twist_pub, long_goal, target_offset_y, target_offset_x

    try:

        for marker in msg.markers:

            if int(marker.id) == current_id:
                rospy.loginfo("Target Found...")
                found = True
                target_offset_y = marker.pose.pose.position.y
                target_offset_x = marker.pose.pose.position.x
                break

        if not found:
            return

    except:

        if not found:
            long_goal = None
        else:
            rospy.loginfo("Lost Target...")

        return

    twist = Twist()

    if abs(target_offset_y) > y_threshold:
        speed = target_offset_y * y_scale
        twist.angular.z = copysign(max(min_angular_speed, min(max_angular_speed, abs(speed))), speed)
    else:
        twist.angular.z = 0

    # if abs(target_offset_x - goal_x) > x_threshold:
    #     speed = (target_offset_x - goal_x) * x_scale
    #     if speed < 0:
    #         speed *= 1.5
    #     twist.linear.x = copysign(min(max_linear_speed, max(min_linear_speed, abs(speed))), speed)
    # else:
    #     twist.linear.x = 0.0
    #     long_goal = None
    #     return

    twist.linear.x = 0.1

    long_goal = twist

    if target_offset_x < 1.0:
        long_goal = None
    return


def odom_callback(msg):
    global current_pose

    current_pose = msg.pose.pose


def get_initial_pose(pose):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = '/odom'
    goal_pose.target_pose.pose.position.x = pose.position.x
    goal_pose.target_pose.pose.position.y = pose.position.y
    goal_pose.target_pose.pose.position.z = pose.position.z
    goal_pose.target_pose.pose.orientation.x = pose.orientation.x
    goal_pose.target_pose.pose.orientation.y = pose.orientation.y
    goal_pose.target_pose.pose.orientation.z = pose.orientation.z
    goal_pose.target_pose.pose.orientation.w = pose.orientation.w

    return goal_pose


def main():
    global cur_pos, twist_pub, client

    rospy.init_node('moving')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['finished'])

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')

    sis.start()

    sm.userdata.sm_counter = 0

    rospy.Subscriber("/joy", Joy, joy_callback)
    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, ar_callback)
    rospy.Subscriber("/odom", Odometry, odom_callback)

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('move_in', move_in(),
                               transitions={'moving': 'move_in',
                                            'arrived': 'rotate_check'})
        smach.StateMachine.add('rotate_check', rotate_check(),
                               transitions={'found': 'rough_close',
                                            'not_found': 'rotate_check',
                                            'finished': 'finished'})
        smach.StateMachine.add('rough_close', rough_close(),
                               transitions={'moving': 'rough_close',
                                            'arrived': 'moving_towards'})

        smach.StateMachine.add('moving_towards', moving_towards(),
                               transitions={'moving': 'moving_towards',
                                            'arrived': 'docking',
                                            'rotating': 'rotate_check'})
        smach.StateMachine.add('docking', docking(),
                               transitions={'finished': 'go_back'})
        smach.StateMachine.add('go_back', go_back(),
                               transitions={'moving': 'rotate_check'})

        # Execute SMACH plan
        outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
