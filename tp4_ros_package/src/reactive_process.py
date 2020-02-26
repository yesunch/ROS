#!/usr/bin/env python
import rospy
import sys
import tty
from kobuki_msgs.msg import BumperEvent
from kobuki_msgs.msg import WheelDropEvent
from geometry_msgs.msg import Twist

#Definition of reactive process node
class Reactive_Node:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        bump_sub = rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, self.stop)
        wheeldrop_sub = rospy.Subscriber("/mobile_base/events/wheel_drop", WheelDropEvent, self.stop)
        cmd_vel = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
        rospy.spin()

    def stop(self):
        # stop turtlebot
        rospy.loginfo("Stop turtlebot")
        cmd_vel.publish(Twist())
        # sleep just make sure the turtlebot receive the stop message
        rospy.sleep(1)


if __name__ == '__main__':
    my_reactive_node = Reactive_Node('my_reactive_node')
