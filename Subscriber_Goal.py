#!/usr/bin/env python
# coding=utf-8
import rospy
import time
from geometry_msgs.msg import PoseStamped, TwistStamped, Point, Vector3, Pose, Quaternion
from autoware_msgs.msg import Lane, Waypoint, DTLane #, WPState
import std_msgs.msg
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import math
from pylab import *
import matplotlib.pyplot as plt
#import analyse_map as analyse
#import csv_reader
import parking_parser

global goal_position
goal_position = (0,0,0)
global flag 
flag = 0

def goal_callback(data):
    global goal_position
    global flag
    flag = 1
    position_x = data.pose.position.x
    position_y = data.pose.position.y
    position_z = data.pose.position.z

    goal_position = (position_x, position_y, position_z)
    print(goal_position)
    print(ret_goal_rec())

def goal():
    rospy.init_node('goal_node', anonymous=True)

    rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_callback)
    
    rospy.spin()

def ret_goal_rec():
    global flag 
    if flag ==0: 
        return 0
    else: 
        return 1

def ret_goal():
    return goal_position

if __name__ == '__main__':
    
    #listener() 
    try:
        #talker()
        goal()
        #listener()
        
    except rospy.ROSInterruptException:
        pass