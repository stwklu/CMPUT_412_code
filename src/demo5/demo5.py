#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from tf.transformations import decompose_matrix, euler_from_quaternion
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import ar_track_alvar.msg
from ros_numpy import numpify
import numpy as np

from kobuki_msgs.msg import Led
from kobuki_msgs.msg import Sound

import smach
import smach_ros

def goal_pose(pose):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]
    return goal_pose


def joy_callback(self, msg):
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

	if msg.button[4] == 1:
		temp_twist.linear.x = 0
		temp_twist.angular.z = 0
	# memorize search origin
	if msg.button[3] == 1:
		search_origin = rospy.wait_for_message('/odom', Odometry)
		search_origin = goal_pose(search_origin)
		sound_pub.publish(0)

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




class move_in(smach.State):
	def __init__(self,outcomes=['moving', 'arrived']):
		pass
	def execute(self, userdata):
		goal = goal_pose(search_origin)
		client.send_goal(goal)
		client.wait_for_result()
		return 'arrived'

class rotate_check(smach.State):
	global found, twist_pub
	def __init__(self,outcomes=['found', 'not_found', 'finished']):
		pass
	def execute(self, userdata):
		found = False
		while not found:
			twist = Twist()
			twist.linear.x = 0
			twist.angular.z = rotate_velocity
			twist_pub.publish(temp_twist)
		if found:
			return 'found'
		else:
			return 'not_found'

class moving_towards(smach.State):
	global long_goal, twist_pub
	def __init__(self,outcomes=['moving', 'arrived']):
		pass
	def execute(self, userdata):
		if found and long_goal:
			twist_pub.publish(twist)

class docking(smach.State):
	"""docstring for docking"""
	def __init__(self, outcomes=['finished']):
		pass
	def execute(self, userdata):
		twist = Twist()
		twist.linear.x = 0
		twist.angular.z = 0
		twist_pub.publish(temp_twist)
		rospy.sleep(3)
		return 'finished'
		

class go_back(smach.State):
	def __init__(self,outcomes=['moving', 'arrived']):
		pass
	def execute(self, userdata):
		return 'moving'

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
        smach.StateMachine.add('move_in', move_in(), 
                               transitions={'moving':'move_in', 
                                            'arrived':'rotate_check'})
        smach.StateMachine.add('rotate_check', rotate_check(), 
                               transitions={'found':'moving_towards',
                               				'not_found': 'rotate_check',
                               				'finished': 'finished'})
        smach.StateMachine.add('moving_towards', moving_towards(), 
		                       transitions={'moving':'moving_towards',
		                       				'arrived': 'arrived'})
        smach.StateMachine.add('docking', docking(), 
		                       transitions={'finished':'go_back'})
        smach.StateMachine.add('go_back', go_back(), 
		                       transitions={'moving':'move_in',
		                       				'arrived': 'rotate_check'})


    # Execute SMACH plan
    outcome = sm.execute()


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

twist_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=1)
sound_pub = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=1)
rospy.Subscriber("/ar_pose_marker", AlvarMarkers, ar_callback)
rospy.Subscriber("/joy", Joy, joy_callback)
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
client.wait_for_server()


if __name__ == '__main__':
    main()