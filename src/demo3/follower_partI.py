#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from tf.transformations import decompose_matrix, euler_from_quaternion
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ros_numpy import numpify
import numpy as np


class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()

        self.integral = 0
        self.previous_error = 0

        self.Kp = - 1 / 700.0
        self.Kd = 1 / 800.0
        self.Ki = - 1 / 500000.0

        # cv2.namedWindow("window", 1)
        # self.image_sub = rospy.Subscriber('usb_cam/image_raw',
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                          Image, self.image_callback)

        rospy.Subscriber("odom", Odometry, self.odom_callback)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                           Twist, queue_size=1)

        self.twist = Twist()

    def odom_callback(self, msg):
        pose_msg = msg.pose.pose
        pose = numpify(pose_msg)
        __, __, angles, translate, __ = decompose_matrix(pose)

        self.cur_pos = translate[0:2]
        self.cur_heading = angles[2]

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # gray color
        lower_yellow = numpy.array([10, 10, 10])
        upper_yellow = numpy.array([255, 255, 250])

        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        h, w, d = image.shape



        print d

        search_top = 3 * h / 4
        search_bot = 3 * h / 4 + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
            # BEGIN CONTROL
            err = cx - w / 2
            self.twist.linear.x = 0.3  # and <= 1.7

            # 400: 0.1, 300: 0.15, 250, 0.2
            # self.twist.angular.z = -float(err) / 250  # and >= 15

            self.integral = self.integral + err * 0.05
            self.derivative = (err - self.previous_error) / 0.05

            self.twist.angular.z = float(err) * self.Kp + (self.Ki * float(self.integral)) + (
                    self.Kd * float(self.derivative))

            self.previous_error = err

            # print self.previous_error

            # print float(err) * self.Kp
            # print self.Kd * float(self.derivative)
            # print self.Ki * float(self.integral)

        self.cmd_vel_pub.publish(self.twist)
        # END CONTROL
        cv2.imshow("window", image)
        cv2.imshow("mask", mask)
        cv2.waitKey(3)


rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
