#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import sys
from nav_msgs.msg import *
from std_msgs.msg import *

print("Planning Process")

class Process_Node:
    def __init__(self, node_name):
        # Constructor
        self.nname = node_name        
        
        rospy.init_node(node_name, anonymous=True)
        
        rospy.Subscriber("/map", OccupancyGrid, callback)
        
        rospy.spin()
    
    
    
    def callback(self, data):
        print("Data received !")
        print(data)
        for i in range(len(data.data)):
            if (i == len(data.data)-1):
                print(i)


if __name__ == '__main__':
    my_process_node = Process_Node("process_node")


# size of file : 153 599
"""
STEPS TO DO :
- create a class architecture
- method to translate from 1D to 2D
- method to translate from 2D to 1D
- get the current position
- get the desired position

- perform the BFS
- create the new map (with path points in black)
- send the map to new topic


"""