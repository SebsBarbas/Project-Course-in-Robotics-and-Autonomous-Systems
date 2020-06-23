#!/usr/bin/env python
"""
Useful resource for py_trees
https://buildmedia.readthedocs.org/media/pdf/py-trees/devel/py-trees.pdf
"""
from __future__ import print_function
# TF Stuff
import tf
from tf.transformations import *
import tf2_geometry_msgs
import tf2_ros
# Useful tools
import py_trees as pt, itertools, py_trees_ros as ptr, rospy
import sys 
import rospy 
import math
import rospy
from crazyflie_driver.msg import Position
from reactive_sequence import RSequence
from actionlib import SimpleActionClient 
import roslib
import cv2 
import numpy as np
from matplotlib import pyplot as plt
from std_msgs.msg import String
from sensor_msgs.msg import Image
from copy import *
# msg
from std_srvs.srv import Empty, SetBool, SetBoolResponse
from geometry_msgs.msg import PoseStamped,TransformStamped,PoseWithCovarianceStamped
from nav_msgs.msg import Path
# Added by us
import Reset as rst
import copy
from ttictoc import TicToc


class pathFollower(pt.behaviour.Behaviour):
    def __init__(self):
        self.termin=False
        self.pathFollower_sub=rospy.Subscriber('/move_base/TrajectoryPlanner', Path, self.callback)
        self.pathFollower_pub = rospy.Publisher('/cf1/hold_pos', PoseStamped, queue_size=2)
        self.start = rospy.Subscriber('cf1/pose', PoseStamped, self.curr_callback)
        self.path = None
        self.curr_pos = None
        super(pathFollower, self).__init__("Follow Path!")
        
    def callback(self,msg):
        self.path = msg.poses
     
    def curr_callback(self, msg):
            self.curr_pos = msg 
            
    def update(self):
        if self.path is not None:
            if len(self.path) == 0: 
                rospy.loginfo("Path follower success")
                return pt.common.Status.SUCCESS
       
            if len(self.path) != 0:    
                #temp = copy.deepcopy(self.path[0]) #save next node
                #self.path[0].pose.position.x = self.curr_pos.pose.position.x #overwrite path x with current x
                #self.path[0].pose.position.y = self.curr_pos.pose.position.y #overwrite path y with current y
                #self.pathFollower_pub.publish(self.path[0])  #first publish only desired rotation

                #t = TicToc()
                #t.tic()
                #rospy.sleep(3)
                #t.toc()
                #rospy.loginfo("this should be 3 seconds: " + str(t.elapsed))

                #self.path[0].pose.position.x = temp.pose.position.x #update path x to desired x
                #self.path[0].pose.position.y = temp.pose.position.y #update path y to desired y
                self.pathFollower_pub.publish(self.path[0])  #then publish desired position as well
                rospy.sleep(1)
                self.path.pop(0)
                rospy.loginfo("path follower running")
                #randomstuff=raw_input('Please input random stuff and press enter to contnue.\n')
                return pt.common.Status.RUNNING
        else:
            rospy.loginfo("Waiting for a path...")
            return pt.common.Status.RUNNING

            
    def terminate(self, new_status):
        if not self.termin == rst.reset:
            rospy.loginfo("Resetting from pathFollower")
            # Execution checker
            self.path = None
            self.curr_pos = None
            self.termin=not self.termin
