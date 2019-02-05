#!/usr/bin/env python
import rospy
from tf.transformations import decompose_matrix, euler_from_quaternion
from ros_numpy import numpify
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

import smach
import smach_ros

from kobuki_msgs.msg import Led

twist_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=1)

led_pub = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)

cur_pos = [0, 0]
cur_heading = 0
current_twist = Twist()

forward_vel = 0.2
angular_vel = 0.5


class moving_forward(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['arrived', 'crashed', 'moving'],
                             input_keys=['b_state', 'stay_pose'],
                             output_keys=['b_state', 'stay_pose'])

    def execute(self, userdata):
        if userdata.b_state == 1:
            userdata.b_state = 0

            current_twist.linear.x = -1 * current_twist.linear.x
            userdata.stay_pose = cur_pos
            return 'crashed'
        elif cur_pos[0] > 3.5:
            return 'arrived'
        else:
            current_twist.linear.x = forward_vel
            current_twist.angular.z = 0
            twist_pub.publish(current_twist)
            led_pub.publish(Led.RED)
            return 'moving'


class backward_rep(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['moving', 'stay'],
                             input_keys=['b_state', 'stay_pose'],
                             output_keys=['b_state', 'stay_pose'])

    def execute(self, userdata):
        if userdata.stay_pose[0] - cur_pos[0] < 0.2:

            current_twist.linear.x = -1 * forward_vel
            current_twist.angular.z = 0

            twist_pub.publish(current_twist)

            return 'moving'
        else:
            current_twist.linear.x = 0
            current_twist.angular.z = 0

            twist_pub.publish(current_twist)

            return 'stay'


class turning_left(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['turning', 'turning_finish'],
                             input_keys=['b_state', 'stay_pose'],
                             output_keys=['b_state', 'stay_pose'])

    def execute(self, userdata):
        if cur_heading < 1.2:
            current_twist.linear.x = 0
            current_twist.angular.z = angular_vel

            twist_pub.publish(current_twist)

            return 'turning'
        else:
            current_twist.linear.x = forward_vel
            current_twist.angular.z = 0

            twist_pub.publish(current_twist)

            userdata.stay_pose = cur_pos

            return 'turning_finish'


class moving_left(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['crashed', 'moving', 'stay'],
                             input_keys=['b_state', 'stay_pose'],
                             output_keys=['b_state', 'stay_pose'])

    def execute(self, userdata):
        if userdata.b_state == 1:
            userdata.b_state = 0
            current_twist.linear.x = -1 * current_twist.linear.x
            userdata.stay_pose = cur_pos

            return 'crashed'
        elif cur_pos[1] - userdata.stay_pose[1] < 0.5:

            current_twist.linear.x = forward_vel
            current_twist.angular.z = 0

            twist_pub.publish(current_twist)

            return 'moving'
        else:
            current_twist.linear.x = 0
            current_twist.angular.z = -1 * angular_vel

            twist_pub.publish(current_twist)

            return 'stay'


class right_rep(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['moving', 'stay'],
                             input_keys=['b_state', 'stay_pose'],
                             output_keys=['b_state', 'stay_pose'])

    def execute(self, userdata):
        if userdata.stay_pose[1] - cur_pos[1] < 0.2:

            current_twist.linear.x = -1 * forward_vel
            current_twist.angular.z = 0

            twist_pub.publish(current_twist)

            return 'moving'
        else:
            current_twist.linear.x = 0
            current_twist.angular.z = 0

            twist_pub.publish(current_twist)

            return 'stay'


class turning_back(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['turning', 'turning_finish'],
                             input_keys=['b_state', 'stay_pose'],
                             output_keys=['b_state', 'stay_pose'])

    def execute(self, userdata):

        if cur_heading > -1.2:
            current_twist.linear.x = 0
            current_twist.angular.z = -1 * angular_vel

            twist_pub.publish(current_twist)

            return 'turning'
        else:
            current_twist.linear.x = forward_vel
            current_twist.angular.z = 0

            twist_pub.publish(current_twist)

            userdata.stay_pose = cur_pos

            return 'turning_finish'


class moving_right(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['moving', 'stay'],
                             input_keys=['b_state', 'stay_pose'],
                             output_keys=['b_state', 'stay_pose'])

    def execute(self, userdata):
        if userdata.stay_pose[1] - cur_pos[1] < 0.8:

            current_twist.linear.x = forward_vel
            current_twist.angular.z = 0

            twist_pub.publish(current_twist)

            return 'moving'
        else:
            current_twist.linear.x = 0
            current_twist.angular.z = angular_vel

            twist_pub.publish(current_twist)

            return 'stay'


class turning_forward(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['turning', 'turning_finish'],
                             input_keys=['b_state', 'stay_pose'],
                             output_keys=['b_state', 'stay_pose'])

    def execute(self, userdata):

        if cur_heading < 0:
            current_twist.linear.x = 0
            current_twist.angular.z = angular_vel

            twist_pub.publish(current_twist)

            return 'turning'
        else:
            current_twist.linear.x = forward_vel
            current_twist.angular.z = 0

            twist_pub.publish(current_twist)

            userdata.stay_pose = cur_pos

            return 'turning_finish'


class turning_right(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['turning', 'turning_finish'],
                             input_keys=['b_state', 'stay_pose'],
                             output_keys=['b_state', 'stay_pose'])

    def execute(self, userdata):
        if cur_heading > 0:
            current_twist.linear.x = 0
            current_twist.angular.z = -1 * angular_vel

            twist_pub.publish(current_twist)

            return 'turning'
        else:
            current_twist.linear.x = forward_vel
            current_twist.angular.z = 0

            twist_pub.publish(current_twist)

            userdata.stay_pose = cur_pos

            return 'turning_finish'


class stop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['overline'],
                             input_keys=['b_state', 'stay_pose'],
                             output_keys=['b_state', 'stay_pose'])

    def execute(self, userdata):
        stationary_twist = Twist()
        twist_pub.publish(stationary_twist)
        led_pub.publish(Led.GREEN)

        return 'overline'


def odom_callback(msg):
    global cur_pos, cur_heading

    pose_msg = msg.pose.pose

    pose = numpify(pose_msg)
    __, __, angles, translate, __ = decompose_matrix(pose)

    cur_pos = translate[0:2]
    cur_heading = angles[2]

    # print("Heading: ", angles[2] * 180 / 3.14159)


def bumper_callback(msg, sm):
    sm.userdata.bumper_state = msg.state


# def joy_callback(msg):
#     global START
#     if msg.buttons[0] == 1:
#         START = True


if __name__ == "__main__":
    rospy.init_node("odom_node")

    sm = smach.StateMachine(outcomes=['success'])

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')

    sis.start()

    sm.userdata.bumper_state = 0

    rospy.Subscriber("odom", Odometry, odom_callback)
    rospy.Subscriber("mobile_base/events/bumper", BumperEvent, bumper_callback, sm)

    # rospy.Subscriber("joy", Joy, joy_callback)

    sm.userdata.current_pose = cur_pos

    rate = rospy.Rate(10)

    current_twist.linear.x = 1

    twist_pub.publish(current_twist)

    led_pub.publish(Led.RED)

    # rospy.spin()
    with sm:
        # Add states to the container
        smach.StateMachine.add('moving_forward', moving_forward(),
                               transitions={'arrived': 'stop',
                                            'crashed': 'backward_rep',
                                            'moving': 'moving_forward'},
                               remapping={'b_state': 'bumper_state',
                                          'stay_pose': 'cur_pos'})

        smach.StateMachine.add('turning_left', turning_left(),
                               transitions={'turning': 'turning_left',
                                            'turning_finish': 'moving_left'},
                               remapping={'b_state': 'bumper_state',
                                          'stay_pose': 'cur_pos'})

        smach.StateMachine.add('moving_left', moving_left(),
                               transitions={'stay': 'turning_right',
                                            'crashed': 'right_rep',
                                            'moving': 'moving_left'},
                               remapping={'b_state': 'bumper_state',
                                          'stay_pose': 'cur_pos'})

        smach.StateMachine.add('turning_back', turning_back(),
                               transitions={'turning': 'turning_back',
                                            'turning_finish': 'moving_right'},
                               remapping={'b_state': 'bumper_state',
                                          'stay_pose': 'cur_pos'})

        smach.StateMachine.add('moving_right', moving_right(),
                               transitions={'stay': 'turning_forward',
                                            'moving': 'moving_right'},
                               remapping={'b_state': 'bumper_state',
                                          'stay_pose': 'cur_pos'})

        smach.StateMachine.add('backward_rep', backward_rep(),
                               transitions={'stay': 'turning_left',
                                            'moving': 'backward_rep'},
                               remapping={'b_state': 'bumper_state',
                                          'stay_pose': 'cur_pos'})

        smach.StateMachine.add('right_rep', right_rep(),
                               transitions={'stay': 'turning_back',
                                            'moving': 'right_rep'},
                               remapping={'b_state': 'bumper_state',
                                          'stay_pose': 'cur_pos'})

        smach.StateMachine.add('turning_forward', turning_forward(),
                               transitions={'turning': 'turning_forward',
                                            'turning_finish': 'moving_forward'},
                               remapping={'b_state': 'bumper_state',
                                          'stay_pose': 'cur_pos'})

        smach.StateMachine.add('turning_right', turning_right(),
                               transitions={'turning': 'turning_right',
                                            'turning_finish': 'moving_forward'},
                               remapping={'b_state': 'bumper_state',
                                          'stay_pose': 'cur_pos'})

        smach.StateMachine.add('stop', stop(),
                               transitions={'overline': 'success'},
                               remapping={'b_state': 'bumper_state',
                                          'stay_pose': 'cur_pos'})

    outcome = sm.execute()
    rospy.spin()
    sis.stop()
