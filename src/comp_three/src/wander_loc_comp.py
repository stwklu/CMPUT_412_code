#!/usr/bin/env python
from math import copysign

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

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
import tf
from sensor_msgs.msg import Joy

from nav_msgs.srv import SetMap
from nav_msgs.msg import OccupancyGrid

integral = 0

previous_error = 0

cur_pos = [0, 0]
cur_heading = 0
current_twist = Twist()

# Three flag to check touching the line or not
stop_line_flag = False
flag_line_flag = False
backing_flag = False

twist_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=1)
led_pub_1 = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)
led_pub_2 = rospy.Publisher('/mobile_base/commands/led2', Led, queue_size=1)

sound_pub = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=1)

max_rotate_vel = 0.5
max_linear_vel = 0.25
degree_ninty = 4.0 / 2

counter_loc1 = 0
counter_loc2 = 0

location_index = 1

#
TRIANGLE = 1
CIRCLE = 2
RECTANGLE = 3
object_type = CIRCLE
current_type = 0

# checking object or not
isChecking = False

# moving back from loc2 or not
is_loc2_backing = False

# the stop times of location 3
loc3_stop_time = 2

# the index of checked object of location 3
loc3_step_index = 1

# to change the size of window, so that it can see the vertical red line
is_finishing_loc2 = False

time_after_stop = 2

# flag of moving forward
moving_after_stop_flag = False

# --------------new global variable-----------------

# some of the waypoints

is_selecting_second_line = False  # stay on line to go to the location 4

is_end_of_line = False  # stop on the end of the line before go to location 4

client = None

current_pose = [(0, 0, 0), (0, 0, 0, 0)]
linear_velocity = 0.15
rotate_velocity = 0.43
current_id = 2

target_pose = None

search_origin = [(0, 0, 0), (0, 0, 0, 0)]
y_threshold = 0.05
x_threshold = 0.05
y_scale = 0.2
x_scale = 0.5
max_angular_speed = 0.4
min_angular_speed = 0.2
max_linear_speed = 0.3
min_linear_speed = 0.1
goal_x = 0.9

long_goal = None
found = False

unmarked_location = None

loc4_task_id = 1

amcl_pose = None

waypoints = [[(4.03520061705, -1.38057946519, 0.0), (0.0, 0.0, -0.641725485096, 0.766934418173)],  # 1
             [(3.36223502017, -1.54772256866, 0.0), (0.0, 0.0, -0.68204527927, 0.731309945937)],  # 2
             [(2.6380713061, -1.69560516301, 0.0), (0.0, 0.0, -0.667204765293, 0.744874352606)],  # 3
             [(1.89158623165, -1.81141808092, 0.0), (0.0, 0.0, -0.67349647024, 0.739190438639)],  # 4
             [(1.25000321697, -2.02173721367, 0.0), (0.0, 0.0, -0.628555672845, 0.777764595578)],  # 5
             [(3.09692334536, -0.53059310309, 0.0), (0.0, 0.0, 0.772863523771, 0.634572276123)],  # 6
             [(2.20892687623, -0.440633218432, 0.0), (0.0, 0.0, 0.76868255071, 0.6396304685)],  # 7
             [(1.11810920579, -1.11707101113, 0.0), (0.0, 0.0, -0.992131524061, 0.125199995865)]]  # 8

line_wayspoints = [[(0.809008239523, 0.0995233932844, 0.0), (0.0, 0.0, -0.107391319377, 0.994216829732)],
                   [(1.19946615995, -0.0487476900574, 0.0), (0.0, 0.0, -0.488053527053, 0.872813699899)]]

center_waypoints = [(2.30430805763, -1.27184294231, 0.0), (0.0, 0.0, 0.0715265400681, 0.997438696896)]

off_ramp_exit_pose = [(3.7551093735, 0.792378819378, 0.0), (0.0, 0.0, 0.823169254763, 0.567796070798)]

ar_sub = None

# new flag for loc2

# To show whether it is moving toward the object
is_moving_loc2 = False
# To show whether it is arrived at the end of the line
is_end_loc2 = False


# location 1 states
class moving_forward(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop', 'flag_line', 'moving'],
                             input_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'stop_flag', 'flag_flag'],
                             output_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'stop_flag', 'flag_flag'])

    def execute(self, userdata):
        global stop_line_flag, flag_line_flag, moving_after_stop_flag
        if stop_line_flag == True:
            stop_line_flag = False

            userdata.cur_time = rospy.Time.now()
            twist_pub.publish(Twist())
            flag_line_flag = False
            return 'stop'

        elif flag_line_flag == True:

            flag_line_flag = False
            userdata.cur_time = rospy.Time.now()

            return 'flag_line'

            # if userdata.cur_time + rospy.Duration(1) < rospy.Time.now():
            #
            #     userdata.cur_time = rospy.Time.now()
            #     userdata.flag_line_flag = cur_heading
            #     temp_twist = Twist()
            #     temp_twist.linear.x = 0
            #     temp_twist.angular.z = max_rotate_vel
            #     twist_pub.publish(temp_twist)
            #     return 'flag_line'
            # else:
            #     twist_pub.publish(current_twist)
            #     return 'moving'
        else:
            twist_pub.publish(current_twist)
            return 'moving'


class stop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['keep', 'recover'],
                             input_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading'],
                             output_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading'])

    def execute(self, userdata):
        global flag_line_flag, loc3_stop_time
        if userdata.cur_time + rospy.Duration(3) > rospy.Time.now():
            # userdata.cur_time = rospy.Time.now()
            twist_pub.publish(Twist())
            flag_line_flag = False
            return 'keep'
        else:
            twist_pub.publish(current_twist)
            userdata.cur_time = rospy.Time.now()
            flag_line_flag = False

            if location_index == 3:
                loc3_stop_time -= 1

            return 'recover'


class moving_after_stop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['moving', 'stop', 'loc4_start'],
                             input_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading'],
                             output_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading'])

    def execute(self, userdata):
        global stop_line_flag, flag_line_flag, is_selecting_second_line
        if userdata.cur_time + rospy.Duration(2.0) > rospy.Time.now():
            twist_pub.publish(current_twist)
            stop_line_flag = False
            flag_line_flag = False
            return 'moving'
        else:
            twist_pub.publish(current_twist)
            userdata.cur_time = rospy.Time.now()
            stop_line_flag = False
            flag_line_flag = False

            if loc3_stop_time == 1:
                is_selecting_second_line = True
                return 'loc4_start'

            return 'stop'


class turning_left(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['moving_a_bit', 'left_turning', 'stop_turning_loc1', 'stop_turning_loc2',
                                       'stop_turning_loc3'],
                             input_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading'],
                             output_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading'])

    def execute(self, userdata):
        global flag_line_flag, isChecking, is_moving_loc2
        if userdata.cur_time + rospy.Duration(1.0) > rospy.Time.now():
            twist_pub.publish(current_twist)
            flag_line_flag = False
            return 'moving_a_bit'

        elif userdata.cur_time + rospy.Duration(degree_ninty / max_rotate_vel + 1.0) > rospy.Time.now():
            temp_twist = Twist()
            temp_twist.linear.x = 0
            temp_twist.angular.z = max_rotate_vel
            twist_pub.publish(temp_twist)
            flag_line_flag = False

            return 'left_turning'
        else:

            if location_index == 1:
                twist_pub.publish(Twist())
                userdata.cur_time = rospy.Time.now()
                return 'stop_turning_loc1'

            elif location_index == 2:
                twist_pub.publish(Twist())
                userdata.cur_time = rospy.Time.now()
                isChecking = True
                is_moving_loc2 = True
                return 'stop_turning_loc2'

            else:
                twist_pub.publish(Twist())
                userdata.cur_time = rospy.Time.now()
                isChecking = True
                return 'stop_turning_loc3'


class checking_object_loc1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['overtime', 'get_sth', 'not_get_sth'],
                             input_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'],
                             output_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'])

    def execute(self, userdata):
        global counter_loc1, isChecking
        if userdata.cur_time + rospy.Duration(10) < rospy.Time.now():
            temp_twist = Twist()
            temp_twist.linear.x = 0
            temp_twist.angular.z = -max_rotate_vel
            twist_pub.publish(temp_twist)
            userdata.cur_time = rospy.Time.now()
            isChecking = False
            return 'overtime'
        elif counter_loc1 > 2:
            print(counter_loc1)
            temp_twist = Twist()
            temp_twist.linear.x = 0
            temp_twist.angular.z = -max_rotate_vel
            twist_pub.publish(temp_twist)
            userdata.cur_time = rospy.Time.now()

            led_pub_1.publish(Led.GREEN)
            led_pub_2.publish(Led.GREEN)
            for i in range(3):
                sound_pub.publish(0)
                rospy.sleep(1)

            counter_loc1 = 0
            isChecking = False
            return 'get_sth'
        elif counter_loc1 == 2:
            print(counter_loc1)

            temp_twist = Twist()
            temp_twist.linear.x = 0
            temp_twist.angular.z = -max_rotate_vel
            twist_pub.publish(temp_twist)
            userdata.cur_time = rospy.Time.now()

            led_pub_1.publish(Led.GREEN)
            sound_pub.publish(0)
            rospy.sleep(1)
            sound_pub.publish(0)

            counter_loc1 = 0
            isChecking = False
            return 'get_sth'
        elif counter_loc1 == 1:
            print(counter_loc1)

            temp_twist = Twist()
            temp_twist.linear.x = 0
            temp_twist.angular.z = -max_rotate_vel
            twist_pub.publish(temp_twist)
            userdata.cur_time = rospy.Time.now()

            led_pub_2.publish(Led.GREEN)
            sound_pub.publish(0)
            counter_loc1 = 0
            isChecking = False
            return 'get_sth'
        else:
            isChecking = True
            return 'not_get_sth'


class turning_back(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['back_turning', 'stop_back'],
                             input_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading'],
                             output_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading'])

    def execute(self, userdata):
        global location_index, isChecking, stop_line_flag, flag_line_flag
        if userdata.cur_time + rospy.Duration((degree_ninty + 0.2) / max_rotate_vel) > rospy.Time.now():
            temp_twist = Twist()
            temp_twist.linear.x = 0
            temp_twist.angular.z = -max_rotate_vel
            twist_pub.publish(temp_twist)
            isChecking = False
            return 'back_turning'
        else:
            twist_pub.publish(Twist())
            userdata.cur_time = rospy.Time.now()
            location_index += 1
            # led_pub_1.publish(Led.BLACK)
            # led_pub_2.publish(Led.BLACK)
            isChecking = False
            stop_line_flag = False
            flag_line_flag = False

            return 'stop_back'


# location 2 states
class moving_loc2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop', 'moving'],
                             input_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'],
                             output_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'])

    def execute(self, userdata):
        global isChecking, is_end_loc2
        if counter_loc2 > 0 and is_end_loc2:

            if counter_loc2 == 1:
                led_pub_2.publish(Led.ORANGE)
                sound_pub.publish(0)
            elif counter_loc2 == 2:
                led_pub_1.publish(Led.ORANGE)
                sound_pub.publish(0)
                sound_pub.publish(0)
            else:
                led_pub_1.publish(Led.ORANGE)
                led_pub_2.publish(Led.ORANGE)
                sound_pub.publish(0)
                sound_pub.publish(0)
                sound_pub.publish(0)

            if userdata.cur_time + rospy.Duration(0.5) > rospy.Time.now():
                temp_twist = Twist()
                temp_twist.linear.x = max_linear_vel
                temp_twist.angular.z = 0
                twist_pub.publish(temp_twist)
                return 'moving'
            else:
                twist_pub.publish(Twist())
                isChecking = False
                userdata.cur_time = rospy.Time.now()

                is_end_loc2 = False

                return 'stop'

        else:
            twist_pub.publish(current_twist)
            userdata.cur_time = rospy.Time.now()
            return 'moving'


class back_dirction(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop', 'rotating'],
                             input_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'],
                             output_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'])

    def execute(self, userdata):
        global is_loc2_backing, is_finishing_loc2
        if userdata.cur_time + rospy.Duration((degree_ninty * 2 - 0.8) / max_rotate_vel) > rospy.Time.now():

            temp_twist = Twist()
            temp_twist.linear.x = 0
            temp_twist.angular.z = max_rotate_vel
            twist_pub.publish(temp_twist)
            return 'rotating'

        else:
            twist_pub.publish(Twist())
            userdata.cur_time = rospy.Time.now()
            is_loc2_backing = True
            is_finishing_loc2 = True
            return 'stop'


class moving_back_loc2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop', 'moving'],
                             input_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'],
                             output_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'])

    def execute(self, userdata):
        global backing_flag, is_loc2_backing, is_finishing_loc2
        if backing_flag:
            if userdata.cur_time + rospy.Duration(2.15) > rospy.Time.now():
                temp_twist = Twist()
                temp_twist.linear.x = max_linear_vel
                temp_twist.angular.z = 0

                twist_pub.publish(temp_twist)
                is_finishing_loc2 = False
                return 'moving'
            else:
                twist_pub.publish(Twist())
                userdata.cur_time = rospy.Time.now()
                backing_flag = False
                is_loc2_backing = False
                is_finishing_loc2 = False
                return 'stop'

        else:
            twist_pub.publish(current_twist)
            userdata.cur_time = rospy.Time.now()
            return 'moving'


class finish_loc2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop', 'rotating'],
                             input_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'],
                             output_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'])

    def execute(self, userdata):
        global location_index, flag_line_flag, loc3_stop_time
        if userdata.cur_time + rospy.Duration((degree_ninty - 0.5) / max_rotate_vel) > rospy.Time.now():

            temp_twist = Twist()
            temp_twist.linear.x = 0
            temp_twist.angular.z = max_rotate_vel
            twist_pub.publish(temp_twist)
            return 'rotating'

        else:
            twist_pub.publish(Twist())
            userdata.cur_time = rospy.Time.now()
            location_index = 3
            flag_line_flag = False
            loc3_stop_time = 2

            # led_pub_1.publish(Led.BLACK)
            # led_pub_2.publish(Led.BLACK)

            return 'stop'


# location 4 states

class moving_on_line(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop'],
                             input_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'stop_flag', 'flag_flag'],
                             output_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'stop_flag', 'flag_flag'])

    def execute(self, userdata):
        global stop_line_flag, flag_line_flag, moving_after_stop_flag, is_end_of_line, is_selecting_second_line, location_index

        # send map and initial pose to reset amcl
        new_map = rospy.wait_for_message("/map", OccupancyGrid)
        # rospy.init_node('set_map_service')
        rospy.wait_for_service('set_map')
        set_map = rospy.ServiceProxy('set_map', SetMap)
        pose_stamped = PoseWithCovarianceStamped()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.header.stamp = rospy.Time.now()



        pose_stamped.pose.pose.position.x = 0
        pose_stamped.pose.pose.position.y = 0
        pose_stamped.pose.pose.position.z = 0
        pose_stamped.pose.pose.orientation.x = 0
        pose_stamped.pose.pose.orientation.y = 0
        pose_stamped.pose.pose.orientation.z = 0
        pose_stamped.pose.pose.orientation.w = 1

        # Finish the remaining line by following waypoints
        set_map(new_map, pose_stamped)

        # pose = PoseWithCovarianceStamped()
        # pose.header.frame_id = "map"
        # pose.pose.pose.position.x = 0.0
        # pose.pose.pose.position.y = 0.0
        # pose.pose.pose.position.z = 0.0
        # # pose.pose.covariance = amcl_pose.covariance
        # pose.pose.pose.orientation.z = 0
        # pose.pose.pose.orientation.w = 1
        # rospy.loginfo(pose)
        #
        # pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        #
        # pub.publish(pose)

        for single_pose in line_wayspoints:
            client.send_goal(self.goal_pose(single_pose))
            client.wait_for_result()

        location_index = 4

        return 'stop'

    def goal_pose(self, pose):
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


# Move to the center of the location 4
class moving_center(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_task1', 'to_task2', 'to_task3', 'back'],
                             input_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'],
                             output_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'])

    def execute(self, userdata):
        client.send_goal(self.goal_pose(center_waypoints))
        client.wait_for_result()

        if loc4_task_id == 1:

            return 'to_task1'
        elif loc4_task_id == 2:

            return 'to_task2'
        elif loc4_task_id == 3:

            return 'to_task3'

        else:
            return 'back'

    def goal_pose(self, pose):
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


class rotate_check(smach.State):
    global found, twist_pub

    def __init__(self):
        smach.State.__init__(self, outcomes=['found', 'not_found'],
                             input_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'],
                             output_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'])

    def execute(self, userdata):
        if long_goal:
            twist = Twist()
            twist_pub.publish(twist)
            return 'found'
        else:

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


class rough_close(smach.State):
    global found, twist_pub

    def __init__(self):
        smach.State.__init__(self, outcomes=['moving', 'arrived'],
                             input_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'],
                             output_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'])

    def execute(self, userdata):

        if long_goal:

            twist_pub.publish(long_goal)
            return 'moving'
        else:
            return 'arrived'


class moving_AR(smach.State):
    global long_goal, twist_pub, found

    def __init__(self):
        smach.State.__init__(self, outcomes=['arrived', 'rotating'],
                             input_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'],
                             output_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'])

    def execute(self, userdata):
        global target_pose, found, loc4_task_id

        target_pose = self.get_goal_pose()
        if target_pose is None:
            found = False
            return 'rotating'
        else:

            client.send_goal(target_pose)
            client.wait_for_result()
            target_pose = None
            loc4_task_id = 2
            ar_sub.unregister()
            return 'arrived'

    def get_goal_pose(self):
        listener = tf.TransformListener()

        mark_frame_id = '/item_' + str(current_id)
        try:
            listener.waitForTransform('/map', mark_frame_id, rospy.Time(), rospy.Duration(4.0))
            (trans, rot) = listener.lookupTransform('/map', mark_frame_id, rospy.Time(0))
        except:

            return None

        euler = tf.transformations.euler_from_quaternion(rot)

        new_rot = tf.transformations.quaternion_from_euler(0.0, 0.0, euler[2])

        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = '/map'
        goal_pose.target_pose.pose.position.x = trans[0]
        goal_pose.target_pose.pose.position.y = trans[1]
        goal_pose.target_pose.pose.position.z = 0.0
        goal_pose.target_pose.pose.orientation.x = 0.0
        goal_pose.target_pose.pose.orientation.y = 0.0
        goal_pose.target_pose.pose.orientation.z = new_rot[2]
        goal_pose.target_pose.pose.orientation.w = new_rot[3]

        return goal_pose


class docking(smach.State):
    """docstring for docking"""

    def __init__(self):
        smach.State.__init__(self, outcomes=['finished'],
                             input_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'],
                             output_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'])

    def execute(self, userdata):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        twist_pub.publish(twist)
        rospy.sleep(3)
        return 'finished'


class moving_unmarked(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['arrived'],
                             input_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'],
                             output_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'])

    def execute(self, userdata):
        global loc4_task_id

        # send waypoint by joysitck
        while not unmarked_location:
            print("waiting for location command.....")
        client.send_goal(self.goal_pose(waypoints[unmarked_location]))
        client.wait_for_result()
        loc4_task_id = 3

        return 'arrived'

    def goal_pose(self, pose):
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


class rotating_capture(smach.State):
    global found, twist_pub

    def __init__(self):
        smach.State.__init__(self, outcomes=['found', 'not_found'],
                             input_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'],
                             output_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'])

    def execute(self, userdata):
        if long_goal:
            twist = Twist()
            twist_pub.publish(twist)
            return 'found'
        else:

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


class shape_close(smach.State):
    global found, twist_pub

    def __init__(self):
        smach.State.__init__(self, outcomes=['moving', 'stop'],
                             input_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'],
                             output_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'])

    def execute(self, userdata):

        if long_goal:
            twist_pub.publish(long_goal)
            return 'found'
        else:

            return 'stop'


class moving_shape(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop'],
                             input_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'],
                             output_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'])

    def execute(self, userdata):
        global target_pose, found, loc4_task_id, waypoints

        l2_dist = numpy.linalg.norm(amcl_pose[0] - waypoints[0][0])
        closest = 0
        for i in range(1, 8):
            if numpy.linalg.norm(amcl_pose[0] - waypoints[0][0]) < l2_dist:
                closest = i
                l2_dist = numpy.linalg.norm(amcl_pose[0] - waypoints[0][0])
        target_pose = waypoints[closest]

        client.send_goal(self.goal_pose(target_pose))
        client.wait_for_result()
        target_pose = None
        loc4_task_id = 4
        return 'stop'

    def goal_pose(self, pose):
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


class back_loc3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop'],
                             input_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'],
                             output_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'])

    def execute(self, userdata):
        global location_index, stop_line_flag, flag_line_flag
        client.send_goal(self.goal_pose(off_ramp_exit_pose))
        client.wait_for_result()
        location_index -= 1

        stop_line_flag = False
        flag_line_flag = False

        return 'stop'

    def goal_pose(self, pose):
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


# location 3 states
class checking_object_loc3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['right_shape', 'wrong_shape', 'stay'],
                             input_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'],
                             output_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'])

    def execute(self, userdata):
        global counter_loc1, isChecking, loc3_step_index
        if userdata.cur_time + rospy.Duration(3) > rospy.Time.now():
            twist_pub.publish(Twist())
            return 'stay'

        elif object_type != current_type:
            temp_twist = Twist()
            temp_twist.linear.x = 0
            temp_twist.angular.z = -max_rotate_vel
            twist_pub.publish(temp_twist)
            userdata.cur_time = rospy.Time.now()

            isChecking = False
            loc3_step_index += 1

            return 'wrong_shape'
        else:

            temp_twist = Twist()
            temp_twist.linear.x = 0
            temp_twist.angular.z = -max_rotate_vel
            twist_pub.publish(temp_twist)
            userdata.cur_time = rospy.Time.now()
            sound_pub.publish(0)

            led_pub_1.publish(Led.BLACK)
            led_pub_2.publish(Led.RED)

            isChecking = False

            return 'right_shape'


class right_turning_back(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop', 'rotating'],
                             input_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'],
                             output_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'])

    def execute(self, userdata):
        global stop_line_flag
        if userdata.cur_time + rospy.Duration(degree_ninty / max_rotate_vel) > rospy.Time.now():

            temp_twist = Twist()
            temp_twist.linear.x = 0
            temp_twist.angular.z = -max_rotate_vel
            twist_pub.publish(temp_twist)
            return 'rotating'

        else:
            twist_pub.publish(Twist())
            userdata.cur_time = rospy.Time.now()
            stop_line_flag = False
            return 'stop'


class moving_terminate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop', 'moving'],
                             input_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'],
                             output_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'])

    def execute(self, userdata):
        if not stop_line_flag:

            twist_pub.publish(current_twist)
            return 'moving'

        else:
            twist_pub.publish(Twist())
            userdata.cur_time = rospy.Time.now()
            sound_pub.publish(1)

            return 'stop'


class main_controller():
    def __init__(self):
        global client, ar_sub
        rospy.init_node('following_line')

        self.sm = smach.StateMachine(outcomes=['success'])

        self.sis = smach_ros.IntrospectionServer('server_name', self.sm, '/SM_ROOT')

        self.sis.start()

        self.bridge = cv_bridge.CvBridge()

        self.integral = 0
        self.previous_error = 0

        self.Kp = - 1 / 200.0
        self.Kd = 1 / 3000.0
        self.Ki = 0.0

        rospy.Subscriber('usb_cam/image_raw', Image, self.usb_image_callback)
        rospy.Subscriber('camera/rgb/image_raw', Image, self.kinect_image_callback)
        ar_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.ar_callback)
        rospy.Subscriber("/joy", Joy, self.joy_callback)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_callback)

        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        self.sm.userdata.current_time = rospy.Time.now()
        self.sm.userdata.current_pose = cur_pos
        self.sm.userdata.current_heading = cur_heading
        self.sm.userdata.current_loc = 1

        self.sm.userdata.stop_flag = stop_line_flag
        self.sm.userdata.flag_flag = flag_line_flag

    def amcl_callback(self, msg):
        global amcl_pose
        amcl_pose = msg.pose

    def joy_callback(self, msg):
        global unmarked_location
        while not unmarked_location:
            rospy.sleep(1)

        if (msg.buttons[0] == 1) and (msg.buttons[5] == 0):  # spot 1
            unmarked_location = 0
        elif (msg.buttons[1] == 1) and (msg.buttons[5] == 0):  # spot 2
            unmarked_location = 1
        elif (msg.buttons[2] == 1) and (msg.buttons[5] == 0):  # spot 3
            unmarked_location = 2
        elif (msg.buttons[3] == 1) and (msg.buttons[5] == 0):  # spot 4
            unmarked_location = 3
        elif (msg.buttons[0] == 1) and (msg.buttons[5] == 1):  # spot 5
            unmarked_location = 4
        elif (msg.buttons[1] == 1) and (msg.buttons[5] == 1):  # spot 6
            unmarked_location = 5
        elif (msg.buttons[2] == 1) and (msg.buttons[5] == 1):  # spot 7
            unmarked_location = 6
        elif (msg.buttons[3] == 1) and (msg.buttons[5] == 1):  # spot 8
            unmarked_location = 7
        else:
            pass

    def ar_callback(self, msg):
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

    def kinect_image_callback(self, msg):
        global stop_line_flag, flag_line_flag, counter_loc1, counter_loc2, object_type, backing_flag, current_type, max_linear_vel, time_after_stop, long_goal
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # kinect red
        lower_red = numpy.array([0, 200, 50])
        upper_red = numpy.array([360, 256, 256])

        if loc3_step_index > 1 and location_index == 3:
            max_linear_vel = 0.2

        if isChecking:
            if location_index == 1:
                mask = cv2.inRange(hsv, lower_red, upper_red)
                im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                for item in contours:
                    area = cv2.contourArea(item)
                    if area > 1000:
                        # peri = cv2.arcLength(item, True)
                        # approx = cv2.approxPolyDP(item, 0.04 * peri, True)
                        # if len(approx) == 4:
                        counter_loc1 += 1

            elif location_index == 2:
                lower_green = numpy.array([65, 60, 60])
                upper_green = numpy.array([170, 256, 256])

                # lower_red = numpy.array([160, 100, 100])
                # upper_red = numpy.array([360, 256, 256])

                red_mask = cv2.inRange(hsv, lower_red, upper_red)
                green_mask = cv2.inRange(hsv, lower_green, upper_green)

                im2, red_contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                im2, green_contours, hierarchy = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                for item in green_contours:

                    if cv2.contourArea(item) > 1000:

                        peri = cv2.arcLength(item, True)
                        approx = cv2.approxPolyDP(item, 0.04 * peri, True)

                        if len(approx) == 3:
                            object_type = TRIANGLE
                        elif len(approx) == 4:
                            object_type = RECTANGLE
                        else:
                            object_type = CIRCLE

                        counter_loc2 = 1

                        for item in red_contours:
                            if cv2.contourArea(item) > 1000:
                                counter_loc2 += 1

                        break
            elif location_index == 3:
                red_mask = cv2.inRange(hsv, lower_red, upper_red)

                if loc3_step_index == 2:
                    h, w, d = image.shape
                    red_mask = red_mask[0:h, 0:(w / 4 * 3)]

                im2, red_contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                for item in red_contours:

                    if cv2.contourArea(item) > 100:

                        cv2.imwrite("test.png", red_mask)

                        peri = cv2.arcLength(item, True)
                        approx = cv2.approxPolyDP(item, 0.04 * peri, True)

                        if len(approx) == 3:
                            current_type = TRIANGLE
                        elif len(approx) == 4:
                            current_type = RECTANGLE
                        else:
                            current_type = CIRCLE

                        if current_type == object_type:
                            break
            elif location_index == 4 and loc4_task_id == 3:
                # lower_green = numpy.array([65, 60, 60])
                # upper_green = numpy.array([170, 256, 256])

                red_mask = cv2.inRange(hsv, lower_red, upper_red)
                im2, green_contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                for item in green_contours:
                    if cv2.contourArea(item) > 100:
                        peri = cv2.arcLength(item, True)
                        approx = cv2.approxPolyDP(item, 0.04 * peri, True)
                        if len(approx) == 3:
                            current_type = TRIANGLE
                        elif len(approx) == 4:
                            current_type = RECTANGLE
                        else:
                            current_type = CIRCLE

                        M = cv2.moments(red_mask)
                        if current_type == object_type and M["m00"] > 0:
                            cX = int(M["m10"] / M["m00"])
                            cY = int(M["m01"] / M["m00"])
                            cv2.circle(image, (cX, cY), 20, (0, 0, 255), -1)
                            h, w, d = image.shape
                            err = cX - w / 2
                            temp_twist = Twist()
                            temp_twist.linear.x = 0.2
                            temp_twist.angular.z = -float(err) / 100
                            long_goal = temp_twist
            else:
                pass

        # red_mask = cv2.inRange(hsv, lower_red, upper_red)
        # cv2.imshow("refer", image)
        # cv2.waitKey(3)

    def usb_image_callback(self, msg):
        global stop_line_flag, flag_line_flag, counter_loc1, counter_loc2, object_type, backing_flag, current_type, max_linear_vel, time_after_stop, is_end_of_line, is_moving_loc2, is_end_loc2
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # white color
        lower_white = numpy.array([0, 0, 170])
        upper_white = numpy.array([360, 30, 255])

        mask = cv2.inRange(hsv, lower_white, upper_white)

        h, w, d = image.shape
        search_top = 3 * h / 4 + 20
        search_bot = 3 * h / 4 + 30
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0

        if is_selecting_second_line:
            im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            left_list = []

            for item in contours:
                area = cv2.contourArea(item)

                if area > 100:
                    (x, y), radius = cv2.minEnclosingCircle(item)
                    center = (int(x), int(y))
                    radius = int(radius)

                    left_list.append(x - radius)

            if len(left_list) == 2:
                larger_one = left_list[0]
                if left_list[0] < left_list[1]:
                    larger_one = left_list[1]

                mask[:, 0:int(larger_one)] = 0
            else:
                pass

        M = cv2.moments(mask)

        if M['m00'] > 0:
            self.cx_white = int(M['m10'] / M['m00'])
            self.cy_white = int(M['m01'] / M['m00'])
            cv2.circle(image, (self.cx_white, self.cy_white), 20, (0, 0, 255), -1)
            # BEGIN CONTROL
            err = self.cx_white - w / 2
            current_twist.linear.x = max_linear_vel  # and <= 1.7

            self.integral = self.integral + err * 0.05
            self.derivative = (err - self.previous_error) / 0.05

            current_twist.angular.z = float(err) * self.Kp + (self.Ki * float(self.integral)) + (
                    self.Kd * float(self.derivative))

            self.previous_error = err
        else:
            if is_selecting_second_line:
                is_end_of_line = True

            if is_moving_loc2:
                is_end_loc2 = True
                is_moving_loc2 = False

        # usb red
        lower_red = numpy.array([0, 100, 100])
        upper_red = numpy.array([360, 256, 256])

        # if loc3_stop_time == 0:
        #     lower_red = numpy.array([0, 150, 50])
        #     upper_red = numpy.array([360, 256, 256])

        mask = cv2.inRange(hsv, lower_red, upper_red)

        h, w, d = image.shape
        if location_index == 3:
            search_top = h - 50
            search_bot = h - 1
        else:
            search_top = h - 40
            search_bot = h - 1

        if is_finishing_loc2:
            search_top = h - 150
            search_bot = h - 1

        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0

        im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if loc3_step_index > 1 and location_index == 3:
            max_linear_vel = 0.2

        if len(contours) > 0:

            for item in contours:
                area = cv2.contourArea(item)

                if area > 5000:
                    M = cv2.moments(item)
                    self.cx_red = int(M['m10'] / M['m00'])
                    self.cy_red = int(M['m01'] / M['m00'])
                    (x, y), radius = cv2.minEnclosingCircle(item)
                    center = (int(x), int(y))
                    radius = int(radius)
                    cv2.circle(image, center, radius, (0, 255, 0), 2)

                    if center[0] + radius > self.cx_white + 30 and center[1] > h - 25:
                        stop_line_flag = True
                    elif center[0] + radius < self.cx_white + 10 and center[1] > h - 25:

                        if loc3_stop_time > 0 and location_index == 3:
                            stop_line_flag = True
                            flag_line_flag = False
                        else:
                            flag_line_flag = True

                    else:
                        pass


                elif area > 1000:
                    M = cv2.moments(item)
                    self.cx_red = int(M['m10'] / M['m00'])
                    self.cy_red = int(M['m01'] / M['m00'])
                    (x, y), radius = cv2.minEnclosingCircle(item)
                    center = (int(x), int(y))
                    radius = int(radius)
                    cv2.circle(image, center, radius, (0, 255, 0), 2)

                    if center[0] + radius < self.cx_white + 20 and center[1] > h - 25:
                        if loc3_stop_time > 0 and location_index == 3:
                            stop_line_flag = True
                            flag_line_flag = False
                        else:
                            flag_line_flag = True

                    if is_loc2_backing:
                        backing_flag = True

                elif area > 1000 and is_loc2_backing:
                    backing_flag = True

        # red_mask = cv2.inRange(hsv, lower_red, upper_red)
        cv2.imshow("refer_dot", image)
        cv2.waitKey(3)

    def odom_callback(self, msg):
        global cur_pos, cur_heading

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

            smach.StateMachine.add('stop', stop(),
                                   transitions={'keep': 'stop',
                                                'recover': 'moving_after_stop'},
                                   remapping={'cur_time': 'current_time',
                                              'cur_pose': 'current_pose',
                                              'cur_loc': 'current_loc',
                                              'cur_heading': 'current_heading'})

            # Add something for location 4
            smach.StateMachine.add('moving_after_stop', moving_after_stop(),
                                   transitions={'moving': 'moving_after_stop',
                                                'stop': 'moving_forward',
                                                'loc4_start': 'moving_on_line'},
                                   remapping={'cur_time': 'current_time',
                                              'cur_pose': 'current_pose',
                                              'cur_loc': 'current_loc',
                                              'cur_heading': 'current_heading'})

            smach.StateMachine.add('turning_left', turning_left(),
                                   transitions={'moving_a_bit': 'turning_left',
                                                'left_turning': 'turning_left',
                                                'stop_turning_loc1': 'checking_object_loc1',
                                                'stop_turning_loc2': 'moving_loc2',
                                                'stop_turning_loc3': 'checking_object_loc3'},
                                   remapping={'cur_time': 'current_time',
                                              'cur_pose': 'current_pose',
                                              'cur_loc': 'current_loc',
                                              'cur_heading': 'current_heading'})

            smach.StateMachine.add('checking_object_loc1', checking_object_loc1(),
                                   transitions={'overtime': 'turning_back',
                                                'get_sth': 'turning_back',
                                                'not_get_sth': 'checking_object_loc1'},
                                   remapping={'cur_time': 'current_time',
                                              'cur_pose': 'current_pose',
                                              'cur_loc': 'current_loc',
                                              'cur_heading': 'current_heading',
                                              'coun_loc1': 'counter_loc1'})

            smach.StateMachine.add('turning_back', turning_back(),
                                   transitions={'back_turning': 'turning_back',
                                                'stop_back': 'moving_forward'},
                                   remapping={'cur_time': 'current_time',
                                              'cur_pose': 'current_pose',
                                              'cur_loc': 'current_loc',
                                              'cur_heading': 'current_heading'})

            # location 2
            smach.StateMachine.add('moving_loc2', moving_loc2(),
                                   transitions={'stop': 'back_dirction',
                                                'moving': 'moving_loc2'},
                                   remapping={'cur_time': 'current_time',
                                              'cur_pose': 'current_pose',
                                              'cur_loc': 'current_loc',
                                              'cur_heading': 'current_heading',
                                              'coun_loc1': 'counter_loc1'})

            smach.StateMachine.add('back_dirction', back_dirction(),
                                   transitions={'stop': 'moving_back_loc2',
                                                'rotating': 'back_dirction'},
                                   remapping={'cur_time': 'current_time',
                                              'cur_pose': 'current_pose',
                                              'cur_loc': 'current_loc',
                                              'cur_heading': 'current_heading',
                                              'coun_loc1': 'counter_loc1'})

            smach.StateMachine.add('moving_back_loc2', moving_back_loc2(),
                                   transitions={'stop': 'finish_loc2',
                                                'moving': 'moving_back_loc2'},
                                   remapping={'cur_time': 'current_time',
                                              'cur_pose': 'current_pose',
                                              'cur_loc': 'current_loc',
                                              'cur_heading': 'current_heading',
                                              'coun_loc1': 'counter_loc1'})

            smach.StateMachine.add('finish_loc2', finish_loc2(),
                                   transitions={'stop': 'moving_forward',
                                                'rotating': 'finish_loc2'},
                                   remapping={'cur_time': 'current_time',
                                              'cur_pose': 'current_pose',
                                              'cur_loc': 'current_loc',
                                              'cur_heading': 'current_heading',
                                              'coun_loc1': 'counter_loc1'})

            # location 4

            smach.StateMachine.add('moving_on_line', moving_on_line(),
                                   transitions={'stop': 'moving_center'},
                                   remapping={'cur_time': 'current_time',
                                              'cur_pose': 'current_pose',
                                              'cur_loc': 'current_loc',
                                              'cur_heading': 'current_heading',
                                              'stop_flag': 'stop_line_flag',
                                              'flag_flag': 'flag_line_flag'})

            smach.StateMachine.add('moving_center', moving_center(),
                                   transitions={'to_task1': 'rotate_check',
                                                'to_task2': 'moving_unmarked',
                                                'to_task3': 'rotating_capture',
                                                'back': 'back_loc3'},
                                   remapping={'cur_time': 'current_time',
                                              'cur_pose': 'current_pose',
                                              'cur_loc': 'current_loc',
                                              'cur_heading': 'current_heading',
                                              'coun_loc1': 'counter_loc1'})

            smach.StateMachine.add('rotate_check', rotate_check(),
                                   transitions={'found': 'rough_close',
                                                'not_found': 'rotate_check'},
                                   remapping={'cur_time': 'current_time',
                                              'cur_pose': 'current_pose',
                                              'cur_loc': 'current_loc',
                                              'cur_heading': 'current_heading',
                                              'coun_loc1': 'counter_loc1'})

            smach.StateMachine.add('rough_close', rough_close(),
                                   transitions={'moving': 'rough_close',
                                                'arrived': 'moving_AR'},
                                   remapping={'cur_time': 'current_time',
                                              'cur_pose': 'current_pose',
                                              'cur_loc': 'current_loc',
                                              'cur_heading': 'current_heading',
                                              'coun_loc1': 'counter_loc1'})

            smach.StateMachine.add('moving_AR', moving_AR(),
                                   transitions={'rotating': 'rotate_check',
                                                'arrived': 'docking'},
                                   remapping={'cur_time': 'current_time',
                                              'cur_pose': 'current_pose',
                                              'cur_loc': 'current_loc',
                                              'cur_heading': 'current_heading',
                                              'coun_loc1': 'counter_loc1'})

            smach.StateMachine.add('docking', docking(),
                                   transitions={'finished': 'moving_center'},
                                   remapping={'cur_time': 'current_time',
                                              'cur_pose': 'current_pose',
                                              'cur_loc': 'current_loc',
                                              'cur_heading': 'current_heading',
                                              'coun_loc1': 'counter_loc1'})

            smach.StateMachine.add('moving_unmarked', moving_unmarked(),
                                   transitions={'arrived': 'docking'},
                                   remapping={'cur_time': 'current_time',
                                              'cur_pose': 'current_pose',
                                              'cur_loc': 'current_loc',
                                              'cur_heading': 'current_heading',
                                              'coun_loc1': 'counter_loc1'})

            smach.StateMachine.add('rotating_capture', rotating_capture(),
                                   transitions={'found': 'shape_close',
                                                'not_found': 'rotating_capture'},
                                   remapping={'cur_time': 'current_time',
                                              'cur_pose': 'current_pose',
                                              'cur_loc': 'current_loc',
                                              'cur_heading': 'current_heading',
                                              'coun_loc1': 'counter_loc1'})

            smach.StateMachine.add('shape_close', shape_close(),
                                   transitions={'moving': 'shape_close',
                                                'stop': 'moving_shape'},
                                   remapping={'cur_time': 'current_time',
                                              'cur_pose': 'current_pose',
                                              'cur_loc': 'current_loc',
                                              'cur_heading': 'current_heading',
                                              'coun_loc1': 'counter_loc1'})

            smach.StateMachine.add('moving_shape', moving_shape(),
                                   transitions={'stop': 'docking'},
                                   remapping={'cur_time': 'current_time',
                                              'cur_pose': 'current_pose',
                                              'cur_loc': 'current_loc',
                                              'cur_heading': 'current_heading',
                                              'coun_loc1': 'counter_loc1'})

            smach.StateMachine.add('back_loc3', back_loc3(),
                                   transitions={'stop': 'moving_forward'},
                                   remapping={'cur_time': 'current_time',
                                              'cur_pose': 'current_pose',
                                              'cur_loc': 'current_loc',
                                              'cur_heading': 'current_heading',
                                              'coun_loc1': 'counter_loc1'})

            # location 3

            smach.StateMachine.add('checking_object_loc3', checking_object_loc3(),
                                   transitions={'stay': 'checking_object_loc3',
                                                'right_shape': 'right_turning_back',
                                                'wrong_shape': 'turning_back'},
                                   remapping={'cur_time': 'current_time',
                                              'cur_pose': 'current_pose',
                                              'cur_loc': 'current_loc',
                                              'cur_heading': 'current_heading',
                                              'coun_loc1': 'counter_loc1'})

            smach.StateMachine.add('right_turning_back', right_turning_back(),
                                   transitions={'stop': 'moving_terminate',
                                                'rotating': 'right_turning_back'},
                                   remapping={'cur_time': 'current_time',
                                              'cur_pose': 'current_pose',
                                              'cur_loc': 'current_loc',
                                              'cur_heading': 'current_heading',
                                              'coun_loc1': 'counter_loc1'})

            smach.StateMachine.add('moving_terminate', moving_terminate(),
                                   transitions={'stop': 'success',
                                                'moving': 'moving_terminate'},
                                   remapping={'cur_time': 'current_time',
                                              'cur_pose': 'current_pose',
                                              'cur_loc': 'current_loc',
                                              'cur_heading': 'current_heading',
                                              'coun_loc1': 'counter_loc1'})

        # Execute SMACH plan
        outcome = self.sm.execute()

        # rate.sleep()

        rospy.spin()
        self.sis.stop()


if __name__ == "__main__":
    mc = main_controller()
    mc.do_controll()
