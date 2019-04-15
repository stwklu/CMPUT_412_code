#!/usr/bin/env python
# from math import copysign
from time import time
import math

import rospy, cv2, cv_bridge, numpy
from tf.transformations import decompose_matrix, euler_from_quaternion
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from ar_track_alvar_msgs.msg import AlvarMarkers
from ros_numpy import numpify

from kobuki_msgs.msg import Led
from kobuki_msgs.msg import Sound
from std_msgs.msg import Empty
from sensor_msgs.msg import LaserScan

import smach
import smach_ros
import actionlib
import tf

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
object_type = TRIANGLE#CIRCLE
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

# some variables of location 4 ----------------------------------------------------------------------------------------
waypoint = [[(1.09432088617, -1.26156089652, 0.0), (0.0, 0.0, -0.999554667159, 0.0298406997387)],  # Fisrt shape
            [(1.90503728349, -1.39927455019, 0.0), (0.0, 0.0, 0.7234492966, 0.690377516471)],  # Second shape
            [(2.69266185289, -1.34115254041, 0.0), (0.0, 0.0, 0.725254219427, 0.68848116692)],  # Third shape
            # [(3.4752548866, 0.696870890367, 0.0), (0.0, 0.0, 0.783639248972, 0.621216168069)],  # exit
            [(3.66651376044, -0.079103062856, 0.0), (0.0, 0.0, 0.66032940594, 0.750976081943)],  # exit
            ]

shape_point = [[(0.841196788942,-1.38057395174,0.0),(0.0,0.0,0.998701636071,0.0509415558091)],
                [(1.65523937105,-1.47980931176,0.0),(0.0,0.0,0.655927297328,0.754824072629)],
                [(2.55618380352,-1.48896220187,0.0),(0.0,0.0,0.665946769253,0.665946769253)]]
client = None
target_id = 2
box_id = 1
box_pose = None
relative_target_pose = None
absolute_target_pose = None
relative_box_pose = None
shape_x = None
shape_y = None
target_square_id = 1
range_ahead = 2.0

# A flag to start detecting shape in location4
is_in_loction4 = False
box_found = False
target_found = False

# params for PID controller for shape parking
anguler_scale = -0.0006
max_angular_speed = 2.0
min_angular_speed = 0.2
image_err_threshold = 0
image_err_back_threshold = 10

reset_odom = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
image_height = 480
image_width = 640

get_shape = False
shape_index = 0


def get_move_pose(pose):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = '/odom'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = 0.0
    goal_pose.target_pose.pose.orientation.x = 0.0
    goal_pose.target_pose.pose.orientation.y = 0.0
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]

    return goal_pose


def get_target_pose():
    listener = tf.TransformListener()

    mark_frame_id = '/target_' + str(target_id)
    try:
        listener.waitForTransform('/odom', mark_frame_id, rospy.Time(), rospy.Duration(4.0))
        (trans, rot) = listener.lookupTransform('/odom', mark_frame_id, rospy.Time(0))
    except:
        return None

    euler = tf.transformations.euler_from_quaternion(rot)
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

    mark_frame_id = '/back_' + str(box_id)
    try:
        listener.waitForTransform('/odom', mark_frame_id, rospy.Time(), rospy.Duration(4.0))
        (trans, rot) = listener.lookupTransform('/odom', mark_frame_id, rospy.Time(0))
    except:

        return None

    euler = tf.transformations.euler_from_quaternion(rot)
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

            loc3_stop_time -= 1

            return 'recover'


class moving_after_stop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['moving', 'stop', 'jump_to_loc4'],
                             input_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading'],
                             output_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading'])

    def execute(self, userdata):
        global stop_line_flag, flag_line_flag
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

            if location_index == 3 and loc3_stop_time == 1:
                twist_pub.publish(Twist())
                timer = time()
                while time() - timer < 1.0:
                    reset_odom.publish(Empty())
                return 'jump_to_loc4'

            else:
                return 'stop'


# -------------------------------------------------------------------------------------------------------------------------------

class initial_docking_loc4(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['arrived'],
                             input_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading'],
                             output_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading'])

    def execute(self, userdata):
        global absolute_target_pose, absolute_box_pose
        timer = time()
        while time() - timer < 2.0:
            reset_odom.publish(Empty())
        rospy.loginfo(str(current_type))
        rospy.sleep(5)
        client.send_goal(get_move_pose(waypoint[1]))
        client.wait_for_result()
        userdata.cur_time = rospy.Time.now()
        absolute_target_pose = None
        absolute_box_pose = None
        return 'arrived'


class detect_target_pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['rotating', 'found', 'timeout'],
                             input_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading'],
                             output_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading'])

    def execute(self, userdata):
        global absolute_target_pose, relative_box_pose, box_found, target_found, is_in_loction4, current_type,shape_x, shape_y

        absolute_target_pose = get_target_pose()
        absolute_box_pose = get_back_pose()

        if (not box_found) or (not target_found):
            if absolute_target_pose != None:
                target_found = True
                led_pub_1.publish(Led.GREEN)
                sound_pub.publish(0)
                rospy.sleep(1)
                relative_box_pose = None
            if absolute_box_pose != None:
                box_found = True
                led_pub_2.publish(Led.RED)
                sound_pub.publish(0)
                rospy.sleep(1)
            # keep rotating if one of box or target not found yet
            twist = Twist()
            twist.angular.z = max_rotate_vel * 4
            twist_pub.publish(twist)
            # userdata.cur_time = rospy.Time.now()

            # if userdata.cur_time + rospy.Duration(40) < rospy.Time.now():
            #     return 'found'

            return 'rotating'

        else:
            twist_pub.publish(Twist())
            # rospy.sleep(5)
            is_in_loction4 = True
            current_type = 0
            shape_x = None
            shape_y = None

            return 'found'

'''
# Deprecated
class rotating_detect_box(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['rotating', 'moving'],
                             input_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading'],
                             output_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading'])

    def execute(self, userdata):
        global is_in_loction4, current_type
        absolute_box_pose = get_back_pose()
        if absolute_box_pose is None:
            rospy.loginfo("looking for box ....")
            twist = Twist()
            twist.angular.z = max_rotate_vel * 2
            twist_pub.publish(twist)
            return 'rotating'

        else:
            # rospy.sleep(2)
            led_pub_1.publish(Led.ORANGE)
            sound_pub.publish(0)
            rospy.sleep(5)
            is_in_loction4 = True
            current_type = 0

            return 'moving'
'''


class rotating_shape(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['stop', 'rotating'],
                             input_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading'],
                             output_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading'])

    def execute(self, userdata):
        global get_shape,shape_index, is_in_loction4
        if not get_shape and shape_index < 3:
            client.send_goal(get_move_pose(shape_point[shape_index]))
            client.wait_for_result()
            
            if current_type == object_type:
                led_pub_1.publish(Led.ORANGE)
                led_pub_2.publish(Led.BLACK)
                sound_pub.publish(0)
                twist_pub.publish(Twist())
                get_shape = True

                is_in_loction4 = False

                return 'stop'

            else:
                shape_index += 1
                return 'rotating'

        else:
            is_in_loction4 = False
            return 'stop'

        # if shape_x is None:
        #     twist = Twist()
        #     twist.angular.z = max_rotate_vel / 2
        #     twist_pub.publish(twist)
        #     return 'rotating'
            

        # else:
        #     led_pub_1.publish(Led.ORANGE)
        #     led_pub_2.publish(Led.BLACK)
        #     sound_pub.publish(0)
        #     twist_pub.publish(Twist())

        #     rospy.sleep(2)
        #     userdata.cur_time = rospy.Time.now()
        #     return 'stop'


        '''
        if object_type == TRIANGLE:

            client.send_goal(get_move_pose(waypoint[0]))

        elif object_type == CIRCLE:

            client.send_goal(get_move_pose(waypoint[1]))

        else:

            client.send_goal(get_move_pose(waypoint[2]))
        client.wait_for_result()
        '''
        # twist = Twist()
        # twist.linear.x = max_linear_vel
        # twist_pub.publish(twist)

            


class goto_shape(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['stop', 'moving'],
                             input_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading'],
                             output_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading'])

    def execute(self, userdata):
        global shape_index

        # kinet image size
        # h, w = 480, 640

        twist = Twist()

        # if range_ahead >= 0.5:
        #     twist.linear.x = 0.1
        #     if abs(shape_x- image_width/2) > image_err_threshold:
        #         anguler_speed = (shape_x- image_width/2) * anguler_scale
        #         twist.angular.z = math.copysign(max(min_angular_speed, min(max_angular_speed, abs(anguler_speed))), anguler_speed)
        #         print("image error: ", shape_x - image_width/2)
        #         print("anguler_speed", anguler_speed)
        #     twist_pub.publish(twist)
        userdata.cur_time = rospy.Time.now()

        #     return 'moving'

        # else:

        while userdata.cur_time + rospy.Duration(0.25 / 0.1) > rospy.Time.now():
            twist = Twist()
            twist.linear.x = 0.1
            twist_pub.publish(twist)

        rospy.sleep(1)

        while userdata.cur_time + rospy.Duration(0.2 / 0.1 + 1 + 0.3 / 0.1) > rospy.Time.now():
            twist = Twist()
            twist.linear.x = -0.1
            twist_pub.publish(twist)

        rospy.sleep(1)


        return 'stop'


class on_ramp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['arrived'],
                             input_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading'],
                             output_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading'])

    def execute(self, userdata):
        global stop_line_flag, flag_line_flag, loc3_stop_time
        client.send_goal(get_move_pose(waypoint[3]))
        client.wait_for_result()
        userdata.cur_time = rospy.Time.now()
        stop_line_flag = False
        flag_line_flag = False
        loc3_stop_time = 1

        return 'arrived'


# -------------------------------------------------------------------------------------------------------------------------------


class turning_left(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['moving_a_bit', 'left_turning', 'stop_turning_loc1', 'stop_turning_loc2',
                                       'stop_turning_loc3'],
                             input_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading'],
                             output_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading'])

    def execute(self, userdata):
        global flag_line_flag, isChecking
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
                # isChecking = True
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

        rospy.sleep(2)
        rospy.loginfo(str(counter_loc1))
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
        if userdata.cur_time + rospy.Duration((degree_ninty * 1.5) / max_rotate_vel) > rospy.Time.now():
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
        global isChecking
        rospy.loginfo(str(range_ahead))
        if range_ahead < 0.7:
            isChecking = True
            twist_pub.publish(Twist())
            rospy.sleep(2)

            if counter_loc2 == 1:
                led_pub_1.publish(Led.ORANGE)
                sound_pub.publish(0)
                twist_pub.publish(Twist())
            elif counter_loc2 == 2:
                led_pub_1.publish(Led.ORANGE)
                led_pub_2.publish(Led.ORANGE)
                sound_pub.publish(0)
                rospy.sleep(1)
                sound_pub.publish(0)
                twist_pub.publish(Twist())
            else:
                led_pub_1.publish(Led.ORANGE)
                led_pub_2.publish(Led.ORANGE)
                sound_pub.publish(0)
                rospy.sleep(1)
                sound_pub.publish(0)
                rospy.sleep(1)
                sound_pub.publish(0)
                twist_pub.publish(Twist())


            isChecking = False
            userdata.cur_time = rospy.Time.now()

            return 'stop'

        else:
            twist_pub.publish(current_twist)
            return 'moving'


class back_dirction(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop', 'rotating'],
                             input_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'],
                             output_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'])

    def execute(self, userdata):
        global is_loc2_backing, is_finishing_loc2
        if userdata.cur_time + rospy.Duration((degree_ninty * 2 + 1.0) / max_rotate_vel) > rospy.Time.now():
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
            rospy.sleep(2)
            return 'stop'


class moving_back_loc2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop', 'moving'],
                             input_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'],
                             output_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'])

    def execute(self, userdata):
        global backing_flag, is_loc2_backing, is_finishing_loc2
        rospy.loginfo("count at location 2: ")
        rospy.loginfo(str(counter_loc2))
        rospy.loginfo("object at location 2: ")
        rospy.loginfo(str(object_type))
        if backing_flag:
            if userdata.cur_time + rospy.Duration(2.3) > rospy.Time.now():
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
        global location_index, loc3_stop_time, loc3_step_index, shape_index, box_found, target_found
        if not stop_line_flag:
            twist_pub.publish(current_twist)
            return 'moving'

        else:
            twist_pub.publish(Twist())
            userdata.cur_time = rospy.Time.now()
            sound_pub.publish(1)
            location_index = 1
            loc3_stop_time = 2
            loc3_step_index = 1
            shape_index = 0
            box_found = False
            target_found = False

            return 'stop'


class main_controller():
    def __init__(self):
        global client
        rospy.init_node('comp4')

        self.sm = smach.StateMachine(outcomes=['success'])

        self.sis = smach_ros.IntrospectionServer('server_name', self.sm, '/SM_ROOT')

        self.sis.start()

        self.bridge = cv_bridge.CvBridge()

        self.integral = 0
        self.previous_error = 0

        self.Kp = - 1 / 200.0
        self.Kd = 1 / 3000.0
        self.Ki = 0.0

        rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        # rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_callback)
        rospy.Subscriber('usb_cam/image_raw', Image, self.usb_image_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("waiting for move_base...")
        client.wait_for_server()
        rospy.loginfo("move base arrived..")

        self.sm.userdata.current_time = rospy.Time.now()
        self.sm.userdata.current_pose = cur_pos
        self.sm.userdata.current_heading = cur_heading
        self.sm.userdata.current_loc = 1

        self.sm.userdata.stop_flag = stop_line_flag
        self.sm.userdata.flag_flag = flag_line_flag

    def scan_callback(self, msg):
        global range_ahead
        range_ahead = msg.ranges[len(msg.ranges)/2]

    def image_callback(self, msg):
        global stop_line_flag, flag_line_flag, counter_loc1, counter_loc2, object_type, backing_flag, current_type, max_linear_vel, time_after_stop, shape_x, shape_y,image_width,image_height

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # lower_red = numpy.array([340, 180, 50])
        # upper_red = numpy.array([359, 255, 255])

        lower_red = numpy.array([100,150,50])
        upper_red = numpy.array([255,255,255])

        lower_green = numpy.array([30, 40, 60])
        upper_green = numpy.array([80, 255, 255])
        #rospy.loginfo("got image...")

        #cv2.imshow("kinet", image)
        #cv2.waitKey(3)

        '''
        red_mask = cv2.inRange(hsv, lower_red, upper_red)
        cv2.imshow("red", red_mask)
        cv2.waitKey(3)

        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        cv2.imshow("green_mask", green_mask)
        cv2.waitKey(3)
        '''

        if isChecking:
            if location_index == 1:
                mask = cv2.inRange(hsv, lower_red, upper_red)
                im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                counter_loc1 = 0
                for item in contours:
                    area = cv2.contourArea(item)
                    if area > 1000:
                        counter_loc1 += 1

            elif location_index == 2:

                red_mask = cv2.inRange(hsv, lower_red, upper_red)
                green_mask = cv2.inRange(hsv, lower_green, upper_green)

                im2, red_contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                im2, green_contours, hierarchy = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                counter_loc2 = 1
                for item in red_contours:
                    if cv2.contourArea(item) > 1000:
                        counter_loc2 += 1

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

                        break
            else:


                red_mask = cv2.inRange(hsv, lower_red, upper_red)
                image_height,image_width = red_mask.shape


                h, w, d = image.shape
                red_mask = red_mask[0:h, 0:(w / 3 * 2)]

                image[0:h, w / 3 * 2:w] = 0

                im2, red_contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                for item in red_contours:

                    if cv2.contourArea(item) > 100:

                        (shape_x, shape_y), shape_radius = cv2.minEnclosingCircle(item)

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

                if shape_x is not None:
                    cv2.circle(image, (int(shape_x), int(shape_y)), 20, (0, 0, 255), -1)


        if is_in_loction4:
            h, w = 480, 640
            red_mask = cv2.inRange(hsv, lower_red, upper_red)
            upper_padding = 80
            lower_padding = 200
            red_mask[0:upper_padding, 0:w] = 0
            red_mask[h-lower_padding:w, 0:w] = 0
            image[0:upper_padding, 0:w] = 0
            image[h-lower_padding:w, 0:w] = 0

            im2, red_contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            for item in red_contours:

                if cv2.contourArea(item) > 1000:

                    peri = cv2.arcLength(item, True)
                    approx = cv2.approxPolyDP(item, 0.04 * peri, True)

                    if len(approx) == 3:
                        current_type = TRIANGLE
                    elif len(approx) == 4:
                        current_type = RECTANGLE
                    else:
                        current_type = CIRCLE

                    if current_type == object_type:
                        (shape_x, shape_y), shape_radius = cv2.minEnclosingCircle(item)
                        break
            if shape_x is not None:
                cv2.circle(image, (int(shape_x), int(shape_y)), 20, (0, 0, 255), -1)

        cv2.imshow("image shape", image)
        cv2.waitKey(3)

    def usb_image_callback(self, msg):
        global stop_line_flag, flag_line_flag, counter_loc1, counter_loc2, object_type, backing_flag, current_type, max_linear_vel, time_after_stop, shape_x, shape_y
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # white color
        lower_white = numpy.array([0, 0, 170])
        upper_white = numpy.array([360, 30, 255])

        mask = cv2.inRange(hsv, lower_white, upper_white)


        if location_index == 2:
            time_after_stop = 4.0
        else:
            time_after_stop = 2.0

        h, w, d = image.shape
        search_top = 3 * h / 4 + 20
        search_bot = 3 * h / 4 + 30
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0

        M = cv2.moments(mask)

        if M['m00'] > 0:
            self.cx_white = int(M['m10'] / M['m00'])
            self.cy_white = int(M['m01'] / M['m00'])
            cv2.circle(image, (self.cx_white, self.cy_white), 20, (0, 0, 255), -1)
            #cv2.circle(mask, (self.cx_white, self.cy_white), 20, (0, 0, 255), -1)
            # BEGIN CONTROL
            err = self.cx_white - w / 2
            current_twist.linear.x = max_linear_vel  # and <= 1.7

            self.integral = self.integral + err * 0.05
            self.derivative = (err - self.previous_error) / 0.05

            current_twist.angular.z = float(err) * self.Kp + (self.Ki * float(self.integral)) + (
                    self.Kd * float(self.derivative))

            self.previous_error = err

        # cv2.imshow("line_follow", image)
        # cv2.waitKey(3)

        lower_red = numpy.array([0, 120, 50])
        upper_red = numpy.array([250, 256, 256])

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
        # cv2.imshow("refer", red_mask)
        # cv2.waitKey(3)

    def odom_callback(self, msg):
        global cur_pos, cur_heading

        pose_msg = msg.pose.pose

        pose = numpify(pose_msg)
        __, __, angles, translate, __ = decompose_matrix(pose)

        cur_pos = translate[0:2]
        cur_heading = angles[2]

    # def ar_callback(self, msg):
    #     global relative_target_pose, relative_box_pose, absolute_target_pose, absolute_box_pose

    #     # absolute_target_pose = get_target_pose()
    #     # absolute_box_pose = get_back_pose()


    #     try:
    #         for marker in msg.markers:

    #             if int(marker.id) == target_id:
    #                 #rospy.loginfo("Target Found...")
    #                 relative_target_pose = marker.pose.pose
    #                 rospy.sleep(5)
    #                 break

    #     except:
    #         relative_target_pose = None

    #     try:
    #         for marker in msg.markers:

    #             if int(marker.id) == box_id:
    #                 rospy.loginfo("Target Found...")
    #                 relative_box_pose = marker.pose.pose
    #                 rospy.sleep(5)

    #                 break

    #     except:
    #         relative_box_pose = None

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

            smach.StateMachine.add('moving_after_stop', moving_after_stop(),
                                   transitions={'moving': 'moving_after_stop',
                                                'stop': 'moving_forward',
                                                'jump_to_loc4': 'initial_docking_loc4'},
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
                                   transitions={'stop': 'moving_forward',
                                                'moving': 'moving_terminate'},
                                   remapping={'cur_time': 'current_time',
                                              'cur_pose': 'current_pose',
                                              'cur_loc': 'current_loc',
                                              'cur_heading': 'current_heading',
                                              'coun_loc1': 'counter_loc1'})

            # location 4

            smach.StateMachine.add('initial_docking_loc4', initial_docking_loc4(),
                                   transitions={'arrived': 'detect_target_pose'},
                                   remapping={'cur_time': 'current_time',
                                              'cur_pose': 'current_pose',
                                              'cur_loc': 'current_loc',
                                              'cur_heading': 'current_heading'})

            smach.StateMachine.add('detect_target_pose', detect_target_pose(),
                                   transitions={'rotating': 'detect_target_pose',
                                                'found': 'rotating_shape',
                                                'timeout': 'rotating_shape'},
                                   remapping={'cur_time': 'current_time',
                                              'cur_pose': 'current_pose',
                                              'cur_loc': 'current_loc',
                                              'cur_heading': 'current_heading'})

            '''
            smach.StateMachine.add('rotating_detect_box', rotating_detect_box(),
                                   transitions={'rotating': 'rotating_detect_box',
                                                'moving': 'rotating_shape'},
                                   remapping={'cur_time': 'current_time',
                                              'cur_pose': 'current_pose',
                                              'cur_loc': 'current_loc',
                                              'cur_heading': 'current_heading'})
                                              '''

            smach.StateMachine.add('rotating_shape', rotating_shape(),
                                   transitions={'stop': 'goto_shape',
                                                'rotating': 'rotating_shape'},
                                   remapping={'cur_time': 'current_time',
                                              'cur_pose': 'current_pose',
                                              'cur_loc': 'current_loc',
                                              'cur_heading': 'current_heading'})

            smach.StateMachine.add('goto_shape', goto_shape(),
                                   transitions={'stop': 'on_ramp',
                                                'moving': 'goto_shape'},
                                   remapping={'cur_time': 'current_time',
                                              'cur_pose': 'current_pose',
                                              'cur_loc': 'current_loc',
                                              'cur_heading': 'current_heading'})

            smach.StateMachine.add('on_ramp', on_ramp(),
                                   transitions={'arrived': 'moving_forward'},
                                   remapping={'cur_time': 'current_time',
                                              'cur_pose': 'current_pose',
                                              'cur_loc': 'current_loc',
                                              'cur_heading': 'current_heading'})

        # Execute SMACH plan
        outcome = self.sm.execute()
        led_pub_1.publish(Led.GREEN)
        led_pub_2.publish(Led.ORANGE)

        # rate.sleep()

        rospy.spin()
        self.sis.stop()


if __name__ == "__main__":
    mc = main_controller()
    mc.do_controll()
