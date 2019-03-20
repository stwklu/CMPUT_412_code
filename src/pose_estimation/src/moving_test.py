#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers
from ros_numpy import numpify
from tf.transformations import decompose_matrix, euler_from_quaternion

import smach
import smach_ros

is_moving = False
twist_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=1)

linear_velocity = 0.15
rotate_velocity = 0.5

temp_twist = Twist()

order_list = []
current_id = 0

rotating_pose = [(0, 0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0)]


def goal_pose(pose):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = '/map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]
    return goal_pose


# if __name__ == '__main__':
#     rospy.init_node('naving_square')
#     rospy.Subscriber("/joy", Joy, joy_callback)
#     client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
#     client.wait_for_server()
#     while not rospy.is_shutdown():
#
#         twist_pub.publish(temp_twist)
#
#         if is_moving and len(order_list) == 3:
#             pass
# for pose in waypoints:
#     goal = goal_pose(pose)
#     client.send_goal(goal)
#     client.wait_for_result()


class moving_around(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['rotating_check', 'moving'])


class moving_toward(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop', 'moving'])

    def execute(self, userdata):
        pass


class main_controller():
    def __init__(self):
        rospy.init_node('moving_tags')

        self.sm = smach.StateMachine(outcomes=['success'])

        self.sis = smach_ros.IntrospectionServer('server_name', self.sm, '/SM_ROOT')

        self.sis.start()

        rospy.Subscriber("/joy", Joy, self.joy_callback)
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_pose_marker_callback)

        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

    def joy_callback(self, msg):
        global is_moving, temp_twist, order_list
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

        if msg.button[0] == 1 and (2 not in order_list):
            order_list.append(2)
        elif msg.button[1] == 1 and (3 not in order_list):
            order_list.append(3)
        elif msg.button[2] == 1 and (4 not in order_list):
            order_list.append(4)
        else:
            pass

        if msg.button[5] == 1:
            is_moving = True

    def ar_pose_marker_callback(self, msg):
        if len(msg.markers) == 0:
            return

        pose_msg = msg.markers[0].pose.pose

        # pose = numpify(pose_msg)
        # __, __, angles, translate, __ = decompose_matrix(pose)
        #
        # cur_pos = translate[0:2]
        # cur_heading = angles[2]
        #
        # print "ar: " + str(angles)


    def odom_callback(self, msg):
        # global cur_pos, cur_heading

        pose_msg = msg.pose.pose

        pose = numpify(pose_msg)
        __, __, angles, translate, __ = decompose_matrix(pose)

        cur_pos = translate[0:2]
        cur_heading = angles[2]

    def do_controll(self):
        # Open the container
        with self.sm:
            # location 1
            smach.StateMachine.add('moving_forward', moving_forward(),
                                   transitions={'stop': 'stop',
                                                'flag_line': 'turning_left',
                                                'moving': 'moving_forward'},
                                   remapping={'cur_time': 'current_time',
                                              'cur_pose': 'current_pose',
                                              'cur_loc': 'current_loc',
                                              'cur_heading': 'current_heading',
                                              'stop_flag': 'stop_line_flag',
                                              'flag_flag': 'flag_line_flag'})

        # Execute SMACH plan
        outcome = self.sm.execute()

        # rate.sleep()

        rospy.spin()
        self.sis.stop()
