#!/usr/bin/env python
import rospy
import numpy as np
import tty
import sys
import select
import termios
from geometry_msgs.msg import Twist

# Definition of class
class Teleoperation_Node:
    def __init__(self, node_name):

	rospy.init_node(node_name)	# creation of the node
	pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)	# creation of the topic : cmd_vel 
	move_cmd = Twist()	# creation of the Twist to store the move commands
	linear_speed = 5.0;	# the linear speed
	angular_speed = 3.0;	# the angular speed
	rate = rospy.Rate(2) 	# Rate of 2hz

	while not rospy.is_shutdown(): # while the user has not pressed crtl+c
		cmd=self.getKey();		
		# Setting the Twist according to the key pressed by the user
		if (cmd == 'f'):		# increase the linear and angular speed
			move_cmd.linear.x = move_cmd.linear.x*1.1
			move_cmd.angular.z = move_cmd.angular.z*1.1
		elif (cmd == 's'):		# decrease the linear and angular speed
			move_cmd.linear.x = move_cmd.linear.x*0.9
			move_cmd.angular.z = move_cmd.angular.z*0.9
		elif (cmd == 'u'):		# Move Forward
			move_cmd.linear.x = linear_speed
			move_cmd.angular.z = 0
		elif (cmd == 'j'):		# Move backward
			move_cmd.linear.x = -linear_speed
			move_cmd.angular.z = 0
		elif (cmd == 'k'):		# Move Clockwise
			move_cmd.linear.x = 0
			move_cmd.angular.z = angular_speed
		elif (cmd == 'h'):		# Move counter-clockwise
			move_cmd.linear.x = 0
			move_cmd.angular.z = -angular_speed
		
		pub.publish(move_cmd)	# publish the move command
		rate.sleep()	# what for 0.5 s
	
	print 'Exiting node ' + rospy.get_name() 	# exit

    # This function reads a single keyboard character from the terminal and returns this character
    def getKey(self):
	# Back-up default terminal settings
	settings = termios.tcgetattr(sys.stdin)

        tty.setraw(sys.stdin.fileno()) # Setting stdio terminal to raw (no need for pressing enter)
        key = sys.stdin.read(1) # Read 1 character 
	
	# Restore default terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	# import pdb
	# pdb.set_trace()
        return key

if __name__ == '__main__':
    myteleopobject = Teleoperation_Node('my_teleoperation_node')
