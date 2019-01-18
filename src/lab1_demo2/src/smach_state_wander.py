#!/usr/bin/env python
# BEGIN ALL
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import smach
from smach import State, StateMachine
import threading

def scan_callback(msg):
  global g_range_ahead
  g_range_ahead = min(msg.ranges)


# 1: driving forward
# 0: spinning
class ForwardState(smach.State):
  #global g_range_ahead, state_change_time
  def __init__(self):
    # state initialization
    smach.State.__init__(self, outcomes=[1,0])

  def execute(self, userdata):
    global g_range_ahead, state_change_time, scan_sub, cmd_vel_pub, rate, stop_thre
    rate.sleep()
    if (g_range_ahead < stop_thre or rospy.Time.now() > state_change_time):
      # transit to spin
      state_change_time = rospy.Time.now() + rospy.Duration(5)
      twist = Twist()
      twist.angular.z = 1
      cmd_vel_pub.publish(twist)
      return 0
    else:
      # continue forward
      twist = Twist()
      twist.linear.x = 1
      cmd_vel_pub.publish(twist)
      return 1

class SpinState(smach.State):
  def __init__(self, outcomes=[1,0]):
    smach.State.__init__(self, outcomes=[1,0])

  def execute(self, userdata):
    global state_change_time, scan_sub, cmd_vel_pub, rate
    rate.sleep()
    if rospy.Time.now() > state_change_time:
      state_change_time = rospy.Time.now() + rospy.Duration(5)
      # transit to foward
      twist = Twist()
      twist.linear.x = 1
      cmd_vel_pub.publish(twist)
      return 1
    else:
      # continue spin
      twist = Twist()
      twist.angular.z = 1
      cmd_vel_pub.publish(twist)
      return 0



def main():
  global g_range_ahead, state_change_time, scan_sub, cmd_vel_pub, rate, stop_thre

  stop_thre = rospy.get_param("stop_threshold")

  g_range_ahead = 1 # anything to start
  scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
  cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
  rospy.init_node('smach_state_wander')
  state_change_time = rospy.Time.now()
  rate = rospy.Rate(10)

  sm = smach.StateMachine(outcomes=[1,0])
  with sm:
    smach.StateMachine.add('forward', ForwardState(),
      transitions={1:'forward',
                    0:'spin'})
    smach.StateMachine.add('spin', SpinState(),
      transitions={1:'forward',
                    0:'spin'})

  #while not rospy.is_shutdown():
  outcome = sm.execute()

if __name__ == '__main__':
  main()
# END ALL