#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Joy

waypoints = [
    [(3.24835373927, -1.40084858646, 0.0), (0.0, 0.0, -0.804220067176, 0.594331627588)],  # top left
    [(1.74468228844, -2.84314082869, 0.0), (0.0, 0.0, 0.989949746297, 0.141419587774)],  # top right
    [(0.945313977309, -1.26111663468, 0.0), (0.0, 0.0, 0.854504163474, 0.519444544302)],  # bottom left
    [(-0.0848826540415, -2.39410610964, 0.0), (0.0, 0.0, -0.798635510047, 0.601815023152)]  # bottom right
]

order_list = []

is_moving = False


def goal_pose(pose):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]
    return goal_pose


def joy_callback(msg):
    global order_list, is_moving

    if (msg.buttons[0] == 1) and (0 not in order_list):
        order_list.append(0)
    elif (msg.buttons[1] == 1) and (1 not in order_list):
        order_list.append(1)
    elif (msg.buttons[2] == 1) and (2 not in order_list):
        order_list.append(2)
    elif (msg.buttons[3] == 1) and (3 not in order_list):
        order_list.append(3)
    else:
        pass

    if msg.axes[5] == 1.0:
        is_moving = (not is_moving)


if __name__ == '__main__':
    rospy.init_node('naving_square')
    rospy.Subscriber("/joy", Joy, joy_callback)
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    while not rospy.is_shutdown():

        if is_moving and len(order_list) == 4:
            for index in order_list:
                goal = goal_pose(waypoints[index])
                client.send_goal(goal)
                client.wait_for_result()
