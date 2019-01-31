#!/usr/bin/env python
import rospy
from roslib import message
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from math import copysign
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy

"""
Citation: Referred the code from :https://github.com/pirobot/rbx1/blob/indigo-devel/rbx1_apps/nodes/follower2.py

"""


class Follower():
    def __init__(self):
        rospy.init_node("follower")
        self.START = False

        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)

        # The goal distance (in meters) to keep between the robot and the person
        self.goal_z = rospy.get_param("~goal_z")

        # How far away from the goal distance (in meters) before the robot reacts
        self.z_threshold = rospy.get_param("~z_threshold")

        # How far away from being centered (x displacement) on the person
        # before the robot reacts
        self.x_threshold = rospy.get_param("~x_threshold")

        # How much do we weight the goal distance (z) when making a movement
        self.z_scale = rospy.get_param("~z_scale")

        # How much do we weight left/right displacement of the person when making a movement
        self.x_scale = rospy.get_param("~x_scale")

        # The maximum rotation speed in radians per second
        self.max_angular_speed = rospy.get_param("~max_angular_speed")

        # The minimum rotation speed in radians per second
        self.min_angular_speed = rospy.get_param("~min_angular_speed")

        # The max linear speed in meters per second
        self.max_linear_speed = rospy.get_param("~max_linear_speed")

        # The minimum linear speed in meters per second
        self.min_linear_speed = rospy.get_param("~min_linear_speed")

        # Slow down factor when stopping
        self.slow_down_factor = rospy.get_param("~slow_down_factor")

        # Initialize the movement command
        self.move_cmd = Twist()

        # Publisher to control the robot's movement
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=5)

        rospy.Subscriber("scan", LaserScan, self.scan_callback)

        # Subscribe to the point cloud
        self.depth_subscriber = rospy.Subscriber('point_cloud', PointCloud2, self.set_cmd_vel, queue_size=5)
        rospy.Subscriber("/joy", Joy, self.joy_callback)

        rospy.loginfo("Subscribing to point cloud...")

        # Wait for the pointcloud topic to become available
        rospy.wait_for_message('point_cloud', PointCloud2)

        rospy.loginfo("Ready to follow!")

    def scan_callback(self, msg):
        self.min_range_ahead = min(msg.ranges)

        # if self.min_range_ahead < 0.4:
        #     self.move_cmd.linear.x = (self.min_range_ahead - 0.4) * self.z_scale

    def joy_callback(self, msg):
        if msg.buttons[0] == 1:
            self.START = True

        if msg.buttons[1] == 1:
            self.START = False

    def set_cmd_vel(self, msg):
        # Initialize the centroid coordinates point count
        x = y = z = n = 0

        # max_i = 0
        # min_i = 0

        # Read in the x, y, z coordinates of all points in the cloud
        for point in point_cloud2.read_points(msg, skip_nans=True):
            pt_x = point[0]
            pt_y = point[1]
            pt_z = point[2]

            x += pt_x
            y += pt_y
            z += pt_z
            n += 1
            #
            # if pt_y > max_i:
            #     max_i = pt_y
            #
            # if pt_y < min_i:
            #     min_i = pt_y

        # print max_i, min_i

        # If we have points, compute the centroid coordinates
        if n:
            x /= n
            y /= n
            z /= n

            print x, y, z

            # Check our movement thresholds
            if (abs(z - self.goal_z) > self.z_threshold):
                # Compute the angular component of the movement
                linear_speed = (z - self.goal_z) * self.z_scale

                # Make sure we meet our min/max specifications
                self.move_cmd.linear.x = copysign(max(self.min_linear_speed,
                                                      min(self.max_linear_speed, abs(linear_speed))), linear_speed)
            else:
                self.move_cmd.linear.x *= self.slow_down_factor

            if (abs(x) > self.x_threshold):
                # Compute the linear component of the movement
                angular_speed = -x * self.x_scale

                # Make sure we meet our min/max specifications
                self.move_cmd.angular.z = copysign(max(self.min_angular_speed,
                                                       min(self.max_angular_speed, abs(angular_speed))), angular_speed)
            else:
                # Stop the rotation smoothly
                # self.move_cmd.angular.z *= self.slow_down_factor
                pass
        else:
            # Stop the robot smoothly
            self.move_cmd.linear.x *= self.slow_down_factor
            # self.move_cmd.angular.z *= self.slow_down_factor
            pass

        # Publish the movement command

        if self.START:
            self.cmd_vel_pub.publish(self.move_cmd)

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")

        # Unregister the subscriber to stop cmd_vel publishing
        self.depth_subscriber.unregister()
        rospy.sleep(1)

        # Send an emtpy Twist message to stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        Follower()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Follower node terminated.")
