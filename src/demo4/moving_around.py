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

from sensor_msgs.msg import Joy

twist_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=1)
linear_velocity = 0.15
rotate_velocity = 0.5


class main_controller():
    def __init__(self):
        rospy.init_node('following_line')

        self.temp_twist = Twist()

        rospy.Subscriber("/joy", Joy, self.joy_callback)

        while not rospy.is_shutdown():
            twist_pub.publish(self.temp_twist)

    def joy_callback(self, msg):
        # if msg.buttons[0] == 1:
        #     pass
        #
        # else

        if msg.axes[5] == 1.0:
            self.temp_twist.linear.x = linear_velocity
            self.temp_twist.angular.z = 0

        elif msg.axes[5] == -1.0:
            self.temp_twist.linear.x = -linear_velocity
            self.temp_twist.angular.z = 0
        elif msg.axes[4] == 1.0:
            self.temp_twist.linear.x = 0
            self.temp_twist.angular.z = rotate_velocity
        elif msg.axes[4] == -1.0:
            self.temp_twist.linear.x = 0
            self.temp_twist.angular.z = -rotate_velocity
        else:
            pass

        if msg.buttons[0] == 1:
            self.temp_twist.linear.x = 0
            self.temp_twist.angular.z = 0


if __name__ == "__main__":
    mc = main_controller()
