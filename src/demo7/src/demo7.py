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
from std_msgs.msg import String
from kobuki_msgs.msg import BumperEvent

import smach
import smach_ros
import actionlib
import tf

# twist_pub = rospy.Publisher("teleop_velocity_smoother/raw_cmd_vel", Twist, queue_size=1)

twist_pub = rospy.Publisher("cmd_vel_mux/input/teleop", Twist, queue_size=1)

client = None

# Global variables
# current_pose = None
linear_velocity = 0.1
rotate_velocity = 0.2
push_velocity = 0.1
steer_velocity = 4
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

box_id = 2
target_id = 3

back_goal = None

current_time = None
rospy.Duration(24 * 60 * 60)

push_time = 10

key = 0

bump = 1
bump_state = 0

current_state = "search_box"


# Search for the ar tag on the box
class search_box(smach.State):
    # rotate to look for tag
    def __init__(self):
        smach.State.__init__(self, outcomes=['running', 'stop'])

    def execute(self, userdata):
        global back_goal
        target_pose = get_goal_pose(box_id)

        if target_pose is None:
            twist = Twist()
            twist.angular.z = rotate_velocity
            twist_pub.publish(twist)
            return 'running'
        else:
            back_goal = get_back_pose(box_id)

            return 'stop'

# Seach for the ar tag at the target position
class search_target(object):
    # Rotate to look for tag
    def __init__(self):
        smach.State.__init__(self, outcomes=['running', 'stop'])

    def execute(self, userdata):
        global back_goal
        target_pose = get_goal_pose(target_id)

        if target_pose is None:
            twist = Twist()
            twist.angular.z = rotate_velocity
            twist_pub.publish(twist)
            return 'running'
        else:
            back_goal = get_back_pose(target_id)

            return 'stop'
        

# find the pose of the outside of the box, then send goal
class move_to_side(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop'])

    def execute(self, userdata):
        client.send_goal(back_goal)
        client.wait_for_result()

        return 'stop'


# Push the box for a horizontal distance until it aligns with the parking spot
class horizontal_push(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['running','stop'])

    def execute(self, userdata):
        global current_time
        target_pose = get_goal_pose()

        if target_pose is None:
            twist = Twist()
            twist.linear.x = -0.2
            twist_pub.publish(twist)
            return 'running'
        else:
            twist = Twist()
            twist.linear.x = 0
            twist_pub.publish(twist)
            client.send_goal(target_pose)
            client.wait_for_result()


            current_time = rospy.Time.now()

            while current_time + rospy.Duration(3.1415926535 * 2/ rotate_velocity) > rospy.Time.now():
            
                twist = Twist()
                twist.angular.z = rotate_velocity
                twist_pub.publish(twist)

            twist = Twist()
            twist_pub.publish(twist)

            current_time = rospy.Time.now()

            return 'stop'

# Move from the side of the box to the back of the box (facing target spot)
class move_to_back(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['running','stop'])

    def execute(userdata):
        pass

# Push the box into the parking spot
class forward_push(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['running', 'stop'])

    def execute(self, userdata):
        global bump, bump_state
        twist = Twist()
        while current_time + rospy.Duration(5.0 / push_velocity) > rospy.Time.now():
            
            twist.linear.x = push_velocity
            if bump == 0 and bump_state == 1:
                twist.angular.z = steer_velocity
            elif bump == 2 and bump_state == 1:
                twist.angular.z = -steer_velocity
            elif bump == 1:
                twist.angular.z = 0
            twist_pub.publish(twist)

            #return 'running'

        #else:
        twist = Twist()

        twist_pub.publish(twist)

        return 'stop'


def ar_kinetic_callback(msg):
    return

# We can use this for PID controller when pushing
def ar_usb_callback(msg):
    return

def bumper_callback(msg):
    global bump, bump_state
    return

def odom_callback(msg):
    pose_msg = msg.pose.pose
    pose = numpify(pose_msg)

    __, __, angles, translate, __ = decompose_matrix(pose)
    # print("Heading: ", angles[2] * 180 / 3.14159)


def get_goal_pose(current_id):
    listener = tf.TransformListener()

    mark_frame_id = '/item_' + str(current_id)
    try:
        #listener.waitForTransform('/odom', mark_frame_id, rospy.Time(), rospy.Duration(4.0))
        #(trans, rot) = listener.lookupTransform('/odom', mark_frame_id, rospy.Time(0))
        ar_msg = rospy.wait_for_message("/ar_pose_marker", AlvarMarkers)
    except:
        return None

    while not ar_msg.markers.id = current_id:
        ar_msg = rospy.wait_for_message("/ar_pose_marker", AlvarMarkers)

    try:
        ar_pose = ar_msg.markers[0].pose
    except:
        print "lost target"

    ar_pose.position.x += 1.0


    #euler = tf.transformations.euler_from_quaternion(rot)

    #new_rot = tf.transformations.quaternion_from_euler(euler[0],0.0, 0.0)

    #new_pose = listener.transformPose('/odom', ar_pose)

    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = '/odom'
    goal_pose.target_pose.pose = ar_pose
    '''
    goal_pose.target_pose.pose.position.x = trans[0] + 0.45
    goal_pose.target_pose.pose.position.y = trans[1]
    goal_pose.target_pose.pose.position.z = 0.0
    goal_pose.target_pose.pose.orientation.x = 0.0
    goal_pose.target_pose.pose.orientation.y = 0.0
    goal_pose.target_pose.pose.orientation.z = new_rot[2]
    goal_pose.target_pose.pose.orientation.w = new_rot[3]
    '''

    return goal_pose


def get_back_pose(current_id):
    listener = tf.TransformListener()

    mark_frame_id = '/back_' + str(current_id)
    try:
        listener.waitForTransform('/odom', mark_frame_id, rospy.Time(), rospy.Duration(4.0))
        (trans, rot) = listener.lookupTransform('/odom', mark_frame_id, rospy.Time(0))
    except:

        return None

    euler = tf.transformations.euler_from_quaternion(rot)


    #print e3
    new_rot = tf.transformations.quaternion_from_euler(0.0, 0.0, 3.1415926535)

    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = '/odom'
    goal_pose.target_pose.pose.position.x = trans[0] + 2
    goal_pose.target_pose.pose.position.y = trans[1]
    goal_pose.target_pose.pose.position.z = 0.0
    goal_pose.target_pose.pose.orientation.x = 0.0
    goal_pose.target_pose.pose.orientation.y = 0.0
    goal_pose.target_pose.pose.orientation.z = new_rot[2]
    goal_pose.target_pose.pose.orientation.w = new_rot[3]

    return goal_pose


def main():
    global cur_pos, twist_pub, client, key

    rospy.init_node('demo6')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['finished'])
    sm.userdata.sm_counter = 0

    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, ar_callback)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, bumper_callback)
    #rospy.Publisher('/keys', String, key_callback)

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("waiting for move_base...")
    client.wait_for_server()
    rospy.loginfo("move base arrived..")

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')

    sis.start()

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('search_box', search_box(),
                               transitions={'running': 'search',
                                            'stop': 'search_target'})
        smach.StateMachine.add('search_target', search_target(),
                               transitions={'running': 'search',
                                            'stop': 'move_to_side'})
        smach.StateMachine.add('move_to_side', move_to_side(),
                               transitions={'stop': 'horizontal_push',
                                            'running':'move_to_side'})
        smach.StateMachine.add('horizontal_push', horizontal_push(),
                               transitions={'running': 'horizontal_push',
                                            'stop': 'move_to_back'})
        smach.StateMachine.add('move_to_back', move_to_back(),
                               transitions={'running': 'move_to_back',
                                            'stop': 'forward_push'})
        smach.StateMachine.add('forward_push', forward_push(),
                               transitions={'running': 'forward_push',
                                            'stop': 'finished'})

    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()