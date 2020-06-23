#!/usr/bin/env python
from __future__ import print_function
import py_trees as pt, itertools
import py_trees as pt, py_trees_ros as ptr, rospy
import rospy 
import sys 
import math
import roslib
import cv2 
import numpy as np
from crazyflie_driver.msg import Position
from matplotlib import pyplot as plt
from ttictoc import TicToc
# Import Transforms and Frames 
import tf
from tf.transformations import *
import tf2_geometry_msgs
import tf2_ros
# Imported by us
import Reset as rst
# Import relevant messages
from geometry_msgs.msg import PoseStamped

# Global stuff
pos_ = PoseStamped()
drone_pos=PoseStamped()
complete=False

class Spin(pt.behaviour.Behaviour):
    def __init__(self, degrees_):
        self.rate = rospy.Rate(10) # 10 Hz
        self.pos_ = PoseStamped()
        self.termin=False
        self.pub = rospy.Publisher('/cf1/hold_pos', PoseStamped, queue_size=2)
        self.dr_pos=rospy.Subscriber('/cf1/pose', PoseStamped, self.pos_callback)
        self.yaw=0
        self.static=False
        self.counter=0
        self.degrees_ = degrees_ 
        self.angle = np.linspace(3, degrees_, 4) # From, To, Amount
        self.angle = np.radians(self.angle)
        super(Spin, self).__init__("Spin!")
        
    def pos_callback(self,msg):
        global drone_pos
        drone_pos=msg
        
    def update(self):
        #Update Stuff
        global complete,drone_pos

        if not complete:
            rospy.loginfo('Spin in progress!')

            if not self.static:
                self.pos_.header.stamp = rospy.Time.now()
                self.pos_.header.frame_id = drone_pos.header.frame_id
                self.pos_.pose.position.x = drone_pos.pose.position.x
                self.pos_.pose.position.y = drone_pos.pose.position.y
                self.pos_.pose.position.z = drone_pos.pose.position.z
                self.pos_.pose.orientation = drone_pos.pose.orientation

                self.q_orig = [self.pos_.pose.orientation.x, self.pos_.pose.orientation.y, self.pos_.pose.orientation.z, self.pos_.pose.orientation.w]
                rospy.sleep(5)
                self.static=True
                
            if math.degrees(self.yaw) < self.degrees_ and self.counter < 10:
                self.yaw = self.angle[self.counter] 
                q_rot = quaternion_from_euler(0.0 ,0.0 , self.yaw)
                q_rot[3]= -q_rot[3]
                q_new = quaternion_multiply(q_rot, self.q_orig)
                self.pos_.pose.orientation.x, self.pos_.pose.orientation.y, self.pos_.pose.orientation.z, self.pos_.pose.orientation.w = (q_new[0],q_new[1],q_new[2],q_new[3])
                self.pub.publish(self.pos_)
                rospy.sleep(0.3) # Maybe need longer sleep irl
                self.counter+=1
                rospy.loginfo('Spin at: ' + str(round(math.degrees(self.yaw),2)))
            else:
                complete=True
                rospy.sleep(5)
            
            rospy.loginfo("Some spinning has been done")
            return pt.common.Status.RUNNING
        elif complete:
            return pt.common.Status.SUCCESS

    def terminate(self, new_status):
        global complete

        if not self.termin == rst.reset:
            rospy.loginfo("Resetting from bt_spin")
            # Execution checker
            self.termin = not self.termin  
            complete = False
            self.static = False
            self.yaw=0
            self.static=False
            self.counter=0 
            self.angle = np.linspace(3, self.degrees_, 4) # From, To, Amount
            self.angle = np.radians(self.angle)