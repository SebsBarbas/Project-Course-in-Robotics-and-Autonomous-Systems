#!/usr/bin/env python
"""
Useful resource for py_trees
https://buildmedia.readthedocs.org/media/pdf/py-trees/devel/py-trees.pdf
"""
from __future__ import print_function
# Import python bt - stuff
import py_trees as pt, itertools
import py_trees as pt, py_trees_ros as ptr, rospy
# import tf - stuff
import tf
from tf.transformations import *
import tf2_geometry_msgs
import tf2_ros
# Import relevant tools
import rospy
import math
import numpy as np
import roslib
import cv2 
import sys 

from crazyflie_driver.msg import Position
from reactive_sequence import RSequence
from actionlib import SimpleActionClient
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty, SetBool, SetBoolResponse
from matplotlib import pyplot as plt
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
 
# Added by us
from bt_spin import *
import bt_spin as spin 
from pathFollower import *
from bt_move import *
from bt_localization import Localize 
import A_star 
from ppClient import Planner
import Reset as rst

# Global stuff
goal_counter=0
drone_pos=PoseStamped()
 

class BehaviourTree(pt.trees.BehaviourTree):
    def __init__(self):
        rospy.loginfo("Initialising behaviour tree")
 
        Spinning=pt.composites.Sequence(
                   name='Spin',
                   children=[Spin(359),gotopos()]
        )     

        InitFlight=pt.composites.Sequence(
                     name='InitFlight',
                     children=[Up(0.35), gotopos()]
         )   

        tree = RSequence(name="Main sequence", children=[InitFlight, Spinning, Localize(), gotopos(), Planner(), gotopos(), pathFollower(), reset_behaviours()])

        super(BehaviourTree, self).__init__(tree)

        # execute the behaviour tree
        rospy.sleep(5)
        self.setup(timeout=10000)
        while not rospy.is_shutdown(): 
            try:
              self.tick_tock(1)    
            except KeyboardInterrupt:
                exit()
 
class reset_behaviours(pt.behaviour.Behaviour):
    '''

    This node resets the state of the behaviour tree,
    so that we can run all conditions if we miss to 
    get the cube to the second table.
    '''
    def __init__(self):
        # become a behaviour
        super(reset_behaviours, self).__init__("Reset if failure")

    def update(self):
        global goal_counter
        rospy.loginfo('---------------Resetting-----------------')
        rospy.loginfo("Resetting from bt_main.reset_behaviours")
        rst.reset = not rst.reset
        rst.nextSign = not rst.nextSign
        return pt.common.Status.SUCCESS  
            
class gotopos(pt.behaviour.Behaviour):
    def __init__(self):
       self.curr_pos=rospy.Subscriber('/cf1/pose', PoseStamped, self.position_callback)
       self.cmd_pos=rospy.Subscriber('/cf1/cmd_position', Position, self.cmd_callback)
       self.termin=False
       self.cmd_position=Position()
       self.current_position=PoseStamped()
       self.dist=1
       
       super(gotopos, self).__init__('gotopos')

    def position_callback(self,msg):
        self.current_position=msg
     

    def cmd_callback(self,msg):
        self.cmd_position=msg
       

    def update(self):
        self.dist=math.sqrt((self.cmd_position.x-self.current_position.pose.position.x)**2+(self.cmd_position.y-self.current_position.pose.position.y)**2+(self.cmd_position.z-self.current_position.pose.position.z)**2)
        if self.dist<0.06:
            rospy.sleep(1)
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.RUNNING
        
    def terminate(self, new_status):
        if not self.termin == rst.reset:
            rospy.loginfo("Resetting from bt_main.gotopos")
            # Execution checker
            self.termin=not self.termin 

# class counter(pt.behaviour.Behaviour):
     
#     """
#     Returns running for n ticks and success thereafter.
#     """
 
#     def __init__(self, n, name):
#         rospy.loginfo("Initialising counter behaviour.")
#         self.termin=False
#         # Counter
#         self.i = 0
#         self.n = n
#         # become a behaviour
#         super(counter, self).__init__(name)
        
#     def update(self):
#         global drone_pos
#         # Increment i
#         self.i += 1
#         # Succeed after count is done
#         return pt.common.Status.FAILURE if self.i <= self.n else pt.common.Status.SUCCESS
 
#     def terminate(self, new_status):
#         if not self.termin == rst.reset:
#             rospy.loginfo("Resetting from bt_main.counter")
#             self.i=0
#             self.termin=not self.termin
  
# class wait(pt.behaviour.Behaviour):
     
    # """
    # Returns running for n ticks and success thereafter.
    # """
 
    # def __init__(self, n, name):
    #     rospy.loginfo("Initialising counter behaviour.")
    #     self.termin=False
    #     # Counter
    #     self.i = 0
    #     self.n = n
    #     # become a behaviour
    #     super(wait, self).__init__(name)
        
    # def update(self):
    #     global drone_pos
    #     # Increment i
    #     self.i += 1
    #     # Succeed after count is done
    #     return pt.common.Status.FAILURE if self.i <= self.n else pt.common.Status.SUCCESS
 
    # def terminate(self, new_status):
    #     if not self.termin == rst.reset:
    #         rospy.loginfo("Resetting from bt_main.wait")
    #         self.i=0
    #         self.termin=not self.termin
class BehaviourTree(pt.trees.BehaviourTree):
    
    def __init__(self):
    
        rospy.loginfo("Initialising behaviour tree")

        Spinning=pt.composites.Sequence(
                name='Spin',          
                children=[ Spin(359),gotopos()]
        )     


        Plan=pt.composites.Sequence(
            name='Plan Path',
            children=[Planner()]
        )

        PathFollow=pt.composites.Sequence(
            name='Follow Path',
            childern=[pathFollower()]
        )


        InitFlight=pt.composites.Sequence(
            name='InitFlight',
            children=[Up(0.35), gotopos()]#[counter(5000, "Up_Counter"), Up(0.35)]#,counter(1000,"Wait")]
        )   



        # NoCopyRoutine = pt.composites.Sequence(
        #         name = 'No copy movement',
        #         children = [Forward(1), gotopos(), Spin()]
        # )

        tree = RSequence(name="Main sequence", children=[InitFlight, Spinning ,Localize(), gotopos(), Planner(), gotopos(), pathFollower(), Spinning, reset_behaviours()])#,counter(1000,"Wait"),Down(),reset_behaviours()])#Up(),reset_behaviours()])#,Up(),Image_2(),Down(),reset_behaviours()])

        #tree = RSequence(name="Main sequence", children=[Up(),counter(1000,"Wait"),Image_1(),Uwp(),stay(),counter(1000,"Wait2"),Image_2(),Matching(),Down(),stay(),counter(1000,"Wait3"),reset_behaviours()])#,counter(1000,"Wait"),Down(),reset_behaviours()])#Up(),reset_behaviours()])#,Up(),Image_2(),Down(),reset_behaviours()])
        #tree = RSequence(name="Main sequence", children=[InitFlight])#, gotopos(), Spin()]) # Localize(), gotopos(),Planner(), gotopos(), pathFollower(), Spinning])#,counter(1000,"Wait"),Down(),reset_behaviours()])#Up(),reset_behaviours()])#,Up(),Image_2(),Down(),reset_behaviours()])
        #tree = RSequence(name="Main sequence", children=[Detect])#[InitFlight,Detect,arucoFollow()])
        super(BehaviourTree, self).__init__(tree)

        # execute the behaviour tree
        rospy.sleep(5)
        self.setup(timeout=10000)
        while not rospy.is_shutdown(): 
            try:
                self.tick_tock(1)    
            except KeyboardInterrupt:
                exit()
if __name__ == "__main__":
    
    rospy.init_node('main_state_machine')
    try:
        BehaviourTree()
    except rospy.ROSInterruptException or KeyboardInterrupt:
        exit()

    rospy.spin()