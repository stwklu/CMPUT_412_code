#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from tf.transformations import decompose_matrix, euler_from_quaternion
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ros_numpy import numpify
import numpy as np
import time


class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()

        self.integral = 0
        self.previous_error = 0

        self.Kp = - 1 / 200.0
        self.Kd = 1 / 3000.0
        self.Ki = - 1 / 500000.0

        self.last_M = 1

        # cv2.namedWindow("window", 1)
        # self.image_sub = rospy.Subscriber('usb_cam/image_raw',
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                          Image, self.image_callback)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                           Twist, queue_size=1)

        self.twist = Twist()

        self.is_stuck = False
        self.stuck_time = rospy.Time.now()

        self.greater_ten = False

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # gray color
        lower_white = numpy.array([0, 0, 220])
        upper_white = numpy.array([360, 30, 255])

        mask = cv2.inRange(hsv, lower_white, upper_white)

        h, w, d = image.shape
        search_top = 3 * h / 4 + 20
        search_bot = 3 * h / 4 + 30
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0

        # print search_top
        # print search_bot

        # mask[0:400, 0:w] = 0
        # mask[415:h, 0:w] = 0

        M = cv2.moments(mask)

        # if M['m00'] == 0:
        #     print 0

        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
            # BEGIN CONTROL
            err = cx - w / 2
            self.twist.linear.x = 0.3  # and <= 1.7

            self.integral = self.integral + err * 0.05
            self.derivative = (err - self.previous_error) / 0.05

            self.twist.angular.z = float(err) * self.Kp + (self.Ki * float(self.integral)) + (
                    self.Kd * float(self.derivative))

            self.previous_error = err

            # check the red tangle
            # gray color
            lower_red = numpy.array([156, 195, 100])
            upper_red = numpy.array([180, 255, 255])

            mask = cv2.inRange(hsv, lower_red, upper_red)

            # cv2.imshow("window", image)
            # cv2.imshow("mask", mask)
            # cv2.waitKey(0)

            # mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)

            im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # print len(contours)

            # cv2.drawContours(image, contours, -1, (0, 255, 0), 3)
            #
            # cv2.imshow("window", image)
            # cv2.imshow("mask", mask)
            # cv2.waitKey(0)

            min_y = 1000

            if len(contours) > 0:

                for item in contours:

                    if item.shape[0] < 10:
                        continue
                    else:

                        # print item.shape

                        self.greater_ten = True

                        for index in range(item.shape[0]):
                            if item[index][0][1] < min_y:
                                min_y = item[index][0][1]

            if cy < min_y and (not self.is_stuck) and self.greater_ten:
                self.cmd_vel_pub.publish(Twist())
                for item in contours:
                    print item.shape
                time.sleep(5)
                self.greater_ten = False
                self.stuck_time = rospy.Time.now()

            if rospy.Time.now() < self.stuck_time + rospy.Duration(5):
                self.is_stuck = True

            else:
                self.is_stuck = False

        self.cmd_vel_pub.publish(self.twist)
        self.greater_ten = False
        # self.cmd_vel_pub.publish(Twist())

        # END CONTROL
        # cv2.imshow("window", image)
        # cv2.imshow("mask", mask)
        # cv2.waitKey(3)


rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
