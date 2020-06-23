#!/usr/bin/env python
# Localize node : This will indicate if we update the tf from map to odom
from __future__ import print_function
import py_trees as pt, itertools
import py_trees as pt, py_trees_ros as ptr, rospy
import rospy 
import sys 
import math
import tf
import roslib
import tf2_geometry_msgs
import tf2_ros
import numpy as np
from crazyflie_driver.msg import Position
# Import Transforms and Frames 
from tf.transformations import *
# Import relevant messages
from geometry_msgs.msg import PoseStamped,TransformStamped
from std_msgs.msg import Bool
import Reset as rst


class Localize(pt.behaviour.Behaviour):
    def __init__(self):
        self.rate = rospy.Rate(10) # 10 Hz
        self.termin=False
        self.decision = Bool()
        self.decision.data=False
        self.localize=rospy.Subscriber('/cf1/localize', Bool, self.cb)
        
        super(Localize, self).__init__("Localize!")

        
    def cb(self,msg):
        self.decision.data = msg.data

    def update(self):
        #Update Stuff
        if self.decision.data:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE


    def terminate(self, new_status):
        if not self.termin == rst.reset:
            rospy.loginfo("Resetting from bt_localization")
            # Execution checker
            self.termin=not self.termin  
            self.decision.data=False
            