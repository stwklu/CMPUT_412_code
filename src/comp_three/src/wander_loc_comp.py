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

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped

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

location_index = 3

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

line_wayspoints = [[(), ()]]

center_waypoints = [(), ()]

square_waypoints = [(), ()]

client = None


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
                isChecking = True
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
        global isChecking
        if counter_loc2 > 0:

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
        smach.State.__init__(self, outcomes=['stop', 'moving'],
                             input_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'stop_flag', 'flag_flag'],
                             output_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'stop_flag', 'flag_flag'])

    def execute(self, userdata):
        global stop_line_flag, flag_line_flag, moving_after_stop_flag, is_end_of_line, is_selecting_second_line
        if is_end_of_line:
            twist = Twist()
            twist_pub.publish(twist)
            return 'stop'
        else:
            twist_pub.publish(current_twist)
            is_end_of_line = False
            is_selecting_second_line = False

            return 'moving'


# Move to the center of the location 4
class moving_center(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['arrived'],
                             input_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'],
                             output_keys=['cur_time', 'cur_pose', 'cur_loc', 'cur_heading', 'coun_loc1'])

    def execute(self, userdata):
        client.send_goal(center_waypoints)
        client.wait_for_result()

        return 'arrived'


class rotate_check(smach.State):
    global found, twist_pub

    def __init__(self):
        smach.State.__init__(self, outcomes=['found', 'not_found', 'finished'])

    def execute(self, userdata):
        pass
        # if current_id <= 4 and long_goal:
        #     twist = Twist()
        #     twist_pub.publish(twist)
        #     return 'found'
        # elif current_id <= 4 and (long_goal is None):
        #
        #     if found:
        #         twist = Twist()
        #         twist_pub.publish(twist)
        #
        #         return 'found'
        #
        #     else:
        #
        #         twist = Twist()
        #         twist.linear.x = 0
        #         twist.angular.z = rotate_velocity
        #         twist_pub.publish(twist)
        #         return 'not_found'
        # else:
        #     return 'finished'


# class rough_close(smach.State):
#     global found, twist_pub
#
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['moving', 'arrived'])
#
#     def execute(self, userdata):
#
#         if long_goal:
#
#             twist_pub.publish(long_goal)
#             return 'moving'
#         else:
#             return 'arrived'

class moving_AR(smach.State):
    global long_goal, twist_pub, found

    def __init__(self):
        smach.State.__init__(self, outcomes=['moving', 'arrived', 'rotating'])

    def execute(self, userdata):
        global target_pose, found

        # target_pose = get_goal_pose()
        if target_pose is None:
            found = False
            return 'rotating'
        else:

            client.send_goal(target_pose)
            client.wait_for_result()
            target_pose = None
            return 'arrived'


class moving_unmarked(smach.State):
    pass


class moving_shape(smach.State):
    pass


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
        global client
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

        # client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # client.wait_for_server()

        self.sm.userdata.current_time = rospy.Time.now()
        self.sm.userdata.current_pose = cur_pos
        self.sm.userdata.current_heading = cur_heading
        self.sm.userdata.current_loc = 1

        self.sm.userdata.stop_flag = stop_line_flag
        self.sm.userdata.flag_flag = flag_line_flag

    def kinect_image_callback(self, msg):
        global stop_line_flag, flag_line_flag, counter_loc1, counter_loc2, object_type, backing_flag, current_type, max_linear_vel, time_after_stop
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
            else:
                pass

        red_mask = cv2.inRange(hsv, lower_red, upper_red)
        cv2.imshow("refer", red_mask)
        cv2.waitKey(3)

    def usb_image_callback(self, msg):
        global stop_line_flag, flag_line_flag, counter_loc1, counter_loc2, object_type, backing_flag, current_type, max_linear_vel, time_after_stop, is_end_of_line
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

                mask[:, 0:larger_one] = 0
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
        cv2.imshow("refer", image)
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
                                   transitions={'stop': 'success',
                                                'moving': 'moving_on_line'},
                                   remapping={'cur_time': 'current_time',
                                              'cur_pose': 'current_pose',
                                              'cur_loc': 'current_loc',
                                              'cur_heading': 'current_heading',
                                              'stop_flag': 'stop_line_flag',
                                              'flag_flag': 'flag_line_flag'})

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
