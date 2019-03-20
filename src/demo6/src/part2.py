#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from tf.transformations import decompose_matrix, euler_from_quaternion
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ar_track_alvar_msgs.msg import AlvarMarkers
from ros_numpy import numpify
import numpy as np
from sensor_msgs.msg import Joy

import smach
import smach_ros


twist_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=1)
rospy.Subscriber("/ar_pose_marker", AlvarMarkers, ar_callback)
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
client.wait_for_server()

# Global variables
current_pose = None
linear_velocity = None
rotate_velocity = None
search_origin = [(0, 0, 0), (0, 0, 0, 0)]
y_threshold = 0.05
x_threshold = 0.05
y_scale = 1.0
x_scale = 0.5
max_angular_speed = 2.0
min_angular_speed = 0.5
max_linear_speed = 0.3
min_linear_speed = 0.1
goal_x = 0.6

long_goal = None


def ar_callback(self, msg):
	global found, twist_pub, long_goal
	try:
		marker = msg.markers[0]
		if not found:
			rospy.loginfo("Target Found...")
		found = True
	except:
		if found:
			rospy.loginfo("Lost Target...")
		found = False
		long_goal = None
		return

	target_offset_y = marker.pose.pose.position.y
	target_offset_x = marker.pose.pose.position.x

	twist = Twist()
	if abs(target_offset_y) > y_threshold:
		speed = target_offset_y * y_scale
		twist.angular.z = copysign(max(min_angular_speed, min(max_angular_speed, abs(speed))), speed)
	else:
		twist.angular.z = 0

	if abs(target_offset_x - goal_x) > x_threshold:
		speed = (target_offset_x - goal_x) * x_scale
		if speed < 0:
			speed *= 1.5
		twist.linear.x = copysign(min(max_linear_speed, max(min_linear_speed, abs(speed))), speed)
	else:
		twist.linear.x = 0.0
		long_goal = None
		return

	long_goal = twist
	return


# Search for the ar tag
class search(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['running', 'stop'])

    def execute(self, userdata):
    	pass

# find the pose of the forward facing tag, then send goal
class move_close(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['running', 'stop'])

    def execute(self, userdata):
    	pass

# refine the pose using the backward facing tag
class move_from_back(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['running', 'stop'])

    def execute(self, userdata):
    	pass

# push
class push(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['running', 'finished'])

    def execute(self, userdata):
    	pass

def main():
	global cur_pos, twist_pub, client
    rospy.init_node('demo5')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['finished'])
    sm.userdata.sm_counter = 0

    self.sis = smach_ros.IntrospectionServer('server_name', self.sm, '/SM_ROOT')

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('search', search(), 
                               transitions={'running':'search', 
                                            'stop':'move_close'})
        smach.StateMachine.add('move_close', move_close(), 
                               transitions={'running':'move_close',
                               				'stop': 'move_from_back'})
        smach.StateMachine.add('move_from_back', move_from_back(), 
		                       transitions={'running':'move_from_back',
		                       				'stop': 'push'})
        smach.StateMachine.add('push', push(), 
		                       transitions={'running':'push',
		                       				'stop': 'finished'})


    # Execute SMACH plan
    outcome = sm.execute()