#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from tf.transformations import decompose_matrix, euler_from_quaternion
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ros_numpy import numpify
import numpy as np

from kobuki_msgs.msg import Led
from kobuki_msgs.msg import Sound

import smach
import smach_ros

class move_in(smach.State):
	def __init__(self,outcomes=['moving', 'arrived']):
		pass
	def execute(self, userdata):
		pass

class rotate_check(smach.State):
	def __init__(self,outcomes=['found', 'not_found', 'finished']):
		pass
	def execute(self, userdata):
		pass

class moving_towards(smach.State):
	def __init__(self,outcomes=['moving', 'arrived']):
		pass
	def execute(self, userdata):
		pass

class docking(smach.State):
	"""docstring for docking"""
	def __init__(self, outcomes=['finished']):
		pass
	def execute(self, userdata):
		pass
		

class go_back(smach.State):
	def __init__(self,outcomes=['moving', 'arrived']):
		pass
	def execute(self, userdata):
		pass

def main():
	global cur_pos, twist_pub
    rospy.init_node('demo5')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['finished'])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('move_in', move_in(, 
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
		                       transitions={'moving':'go_back',
		                       				'arrived': 'rotate_check'})


    # Execute SMACH plan
    outcome = sm.execute()


# Global variables
twist_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=1)
current_pose = None


if __name__ == '__main__':
    main()
