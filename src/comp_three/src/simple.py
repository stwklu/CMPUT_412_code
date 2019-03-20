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

current_twist = Twist()
total_redline = 0

rospy.init_node('following_line')

twist_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=1)
led_pub_1 = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)
led_pub_2 = rospy.Publisher('/mobile_base/commands/led2', Led, queue_size=1)
sound_pub = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=1)

client = None

amcl_pose = None

waypoints = [[(4.18814842748, -1.52617745052, 0.0), (0.0, 0.0, -0.620062249254, 0.784552615859)],  # 1
             [(3.53467862035, -1.64303254331, 0.0), (0.0, 0.0, -0.697190052028, 0.716886344795)],  # 2
             [(2.76150910754, -1.73896360769, 0.0), (0.0, 0.0, -0.714845689844, 0.6992822318)],  # 3
             [(1.91072280034, -1.87523495095, 0.0), (0.0, 0.0, -0.692666326679, 0.721258178383)],  # 4
             [(1.12935940772, -1.99085897246, 0.0), (0.0, 0.0, -0.708998752228, 0.705209734291)],  # 5
             [(2.97465200095, -0.301348417754, 0.0), (0.0, 0.0, 0.716908731207, 0.697167032439)],  # 6
             [(2.25758047137, -0.408456817217, 0.0), (0.0, 0.0, 0.801188509568, 0.598412042105)],  # 7
             [(1.11302899287, -1.06059012152, 0.0), (0.0, 0.0, -0.994903567096, 0.100831008027)]]  # 8

line_wayspoints = [[(0.809008239523, 0.0995233932844, 0.0), (0.0, 0.0, -0.107391319377, 0.994216829732)],
                   [(1.19946615995, -0.0487476900574, 0.0), (0.0, 0.0, -0.488053527053, 0.872813699899)]]

center_waypoints = [(2.30430805763, -1.27184294231, 0.0), (0.0, 0.0, 0.0715265400681, 0.997438696896)]

off_ramp_exit_pose = [(3.7551093735, 0.792378819378, 0.0), (0.0, 0.0, 0.823169254763, 0.567796070798)]

ar_sub = None

current_id = 2

found = False
long_goal = None
target_offset_y = 0
target_offset_x = 0

unmarked_location = 0

current_time = rospy.Time.now()


# from  start to location 4
class go_to_loc_4(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['running', 'stop'])

    def execute(self, userdata):
        if total_redline < 4:
            twist_pub.publish(current_twist)
            return 'running'

        else:
            twist = Twist()
            twist_pub.publish(twist)
            return 'stop'


# at the initial pose of location 4
class init_map(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop'])

    def execute(self, userdata):
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

        pose_stamped.pose.covariance = amcl_pose.covariance

        # Finish the remaining line by following waypoints
        set_map(new_map, pose_stamped)

        for single_pose in line_wayspoints:
            client.send_goal(self.goal_pose(single_pose))
            client.wait_for_result()

        client.send_goal(self.goal_pose(center_waypoints))
        client.wait_for_result()

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


# ARtag task
class ar_tag(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['running', 'stop'])

    def execute(self, userdata):

        target_pose = self.get_goal_pose()

        if target_pose == None:
            temp_twist = Twist()
            temp_twist.linear.x = 0
            temp_twist.angular.z = 0.35
            twist_pub.publish(temp_twist)
            return 'running'

        elif long_goal:
            twist_pub.publish(long_goal)
            return 'running'

        else:
            client.send_goal(target_pose)
            client.wait_for_result()

            ar_sub.unregister()

            return 'stop'

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


# # random spot task
class random_spot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop'])

    def execute(self, userdata):
        client.send_goal(self.goal_pose(waypoints[unmarked_location]))
        client.wait_for_result()

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


# go back to white line
class back_to_line(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['running', 'stop'])

    def execute(self, userdata):
        global current_time
        client.send_goal(self.goal_pose(off_ramp_exit_pose))
        client.wait_for_result()

        current_time = rospy.Time.now()

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


# at white line after location 4
class go_to_end(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['moving', 'success'])

    def execute(self, userdata):

        if current_time + rospy.Duration(8) > rospy.Time.now():

            twist_pub.publish(current_twist)

            return 'moving'
        else:

            twist = Twist()
            twist_pub.publish(twist)

            return 'success'


class CallBackClass(object):
    """docstring for ClassName"""

    def __init__(self):
        global ar_sub
        
        self.sm = smach.StateMachine(outcomes=['success'])
        self.sis = smach_ros.IntrospectionServer('server_name', self.sm, '/SM_ROOT')

        self.sis.start()

        with self.sm:
            smach.StateMachine.add('go_to_loc_4', go_to_loc_4(),
                                   transitions={'stop': 'init_map',
                                                'running': 'go_to_loc_4'})
            smach.StateMachine.add('init_map', init_map(),
                                   transitions={'stop': 'ar_tag'})
            smach.StateMachine.add('ar_tag', ar_tag(),
                                   transitions={'stop': 'random_spot',
                                                'running': 'ar_tag'})
            smach.StateMachine.add('random_spot', random_spot(),
                                   transitions={'stop': 'back_to_line'})
            smach.StateMachine.add('go_to_end', go_to_end(),
                                   transitions={'moving': 'go_to_end',
                                                'success': 'success'})

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

    def amcl_callback(self, msg):
        global amcl_pose
        amcl_pose = msg.pose

    def joy_callback(self, msg):
        global unmarked_location
        if (msg.buttons[0] == 1):  # spot 1
            unmarked_location = 0
        elif (msg.buttons[1] == 1):  # spot 2
            unmarked_location = 1
        elif (msg.buttons[2] == 1):  # spot 3
            unmarked_location = 2
        elif (msg.buttons[3] == 1):  # spot 4
            unmarked_location = 3
        elif (msg.buttons[4] == 1):  # spot 5
            unmarked_location = 4
        elif (msg.buttons[5] == 1):  # spot 6
            unmarked_location = 5
        elif (msg.buttons[6] == 1):  # spot 7
            unmarked_location = 6
        elif (msg.buttons[7] == 1):  # spot 8
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

        if abs(target_offset_y) > 0.05:
            speed = target_offset_y * 0.2
            twist.angular.z = copysign(max(0.2, min(0.3, abs(speed))), speed)
        else:
            twist.angular.z = 0

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

    def usb_image_callback(self, msg):
        global stop_line_flag, flag_line_flag, counter_loc1, counter_loc2, object_type, backing_flag, current_type, max_linear_vel, time_after_stop, is_end_of_line, is_moving_loc2, is_end_loc2, total_redline
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

        M = cv2.moments(mask)

        if M['m00'] > 0:
            self.cx_white = int(M['m10'] / M['m00'])
            self.cy_white = int(M['m01'] / M['m00'])
            cv2.circle(image, (self.cx_white, self.cy_white), 20, (0, 0, 255), -1)
            # BEGIN CONTROL
            err = self.cx_white - w / 2
            current_twist.linear.x = 0.3  # and <= 1.7

            self.integral = self.integral + err * 0.05
            self.derivative = (err - self.previous_error) / 0.05

            current_twist.angular.z = float(err) * self.Kp + (self.Ki * float(self.integral)) + (
                    self.Kd * float(self.derivative))

            self.previous_error = err

        # usb red
        lower_red = numpy.array([0, 100, 100])
        upper_red = numpy.array([360, 256, 256])

        # if loc3_stop_time == 0:
        #     lower_red = numpy.array([0, 150, 50])
        #     upper_red = numpy.array([360, 256, 256])

        mask = cv2.inRange(hsv, lower_red, upper_red)

        h, w, d = image.shape

        search_top = h - 40
        search_bot = h - 1

        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0

        im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

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

                    total_redline += 1

                    rospy.sleep(2)


                elif area > 1000:
                    M = cv2.moments(item)
                    self.cx_red = int(M['m10'] / M['m00'])
                    self.cy_red = int(M['m01'] / M['m00'])
                    (x, y), radius = cv2.minEnclosingCircle(item)
                    center = (int(x), int(y))
                    radius = int(radius)
                    cv2.circle(image, center, radius, (0, 255, 0), 2)

                    total_redline += 1

                    rospy.sleep(2)

                else:
                    pass

        # red_mask = cv2.inRange(hsv, lower_red, upper_red)
        cv2.imshow("refer_dot", image)
        cv2.waitKey(3)

    def execute(self):
        outcome = self.sm.execute()
        rospy.spin()
        self.sis.stop()

c = CallBackClass()
c.execute()