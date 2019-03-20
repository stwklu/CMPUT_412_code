#!/usr/bin/env python
import rospy 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan



cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=5)
rospy.init_node('wander')

while not rospy.is_shutdown():
    twist = Twist()
    twist.linear.x = 0.1
    cmd_vel_pub.publish(twist)
