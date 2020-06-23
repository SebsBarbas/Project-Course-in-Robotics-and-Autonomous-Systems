#!/usr/bin/env python
from __future__ import print_function
import py_trees as pt, itertools
import py_trees as pt, py_trees_ros as ptr, rospy
import rospy 
import sys 
import math
import tf
import roslib
import cv2 
import numpy as np
from matplotlib import pyplot as plt
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# Import relevant messages
from geometry_msgs.msg import PoseStamped, Twist
from crazyflie_driver.msg import Position
# Import Transforms and Frames 
from tf.transformations import euler_from_quaternion
import tf2_geometry_msgs
import tf2_ros
import Reset as rst

drone_pos=PoseStamped()

goal=PoseStamped()

class Up(pt.behaviour.Behaviour):
    def __init__(self,n):
        # Create Publisher
        self.pub_cmd = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
        # Create Subscriber to keyboard
        self.dr_pos=rospy.Subscriber('/cf1/pose', PoseStamped, self.pos_callback)
        self.termin=False    
        self.move_done=False
        self.height=n
        self.hold_pose = rospy.Publisher('/cf1/hold_pos_cmd', Position, queue_size=2)
        self.received_message = False
        
        super(Up, self).__init__("Up!")
        
    def pos_callback(self,msg):
        global drone_pos
        drone_pos=msg
        self.received_message = True
        
    def pub(self,goal):
        cmd = Position()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = goal.header.frame_id
        cmd.x = goal.pose.position.x
        cmd.y = goal.pose.position.y
        cmd.z = goal.pose.position.z
        _, _, yaw = euler_from_quaternion((goal.pose.orientation.x,
                                            goal.pose.orientation.y,
                                            goal.pose.orientation.z,
                                            goal.pose.orientation.w))
        cmd.yaw = math.degrees(yaw)
        self.hold_pose.publish(cmd)
        
    def update(self):
        global  drone_pos, goal

        if self.received_message == True:
            if self.move_done == False:   
                goal.header.stamp = rospy.Time.now()
                goal.header.frame_id  =   drone_pos.header.frame_id
                goal.pose.position.y = drone_pos.pose.position.y
                goal.pose.position.x = drone_pos.pose.position.x
                goal.pose.position.z = drone_pos.pose.position.z+self.height
                goal.pose.orientation= drone_pos.pose.orientation
                self.move_done=True
                self.pub(goal)
            
                return pt.common.Status.RUNNING
            else:
                return pt.common.Status.SUCCESS
        else:
                return pt.common.Status.FAILURE

    def terminate(self, new_status): 
        if not self.termin == rst.reset:
            rospy.loginfo("Resetting from bt_move.Up")
            # Execution checker
            self.termin=not self.termin  
            self.move_done=False

class Forward(pt.behaviour.Behaviour):
        
    def __init__(self,n):
        # Create Publisher
        self.pub_cmd = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
        # Create Subscriber to keyboard
        self.dr_pos=rospy.Subscriber('/cf1/pose', PoseStamped, self.pos_callback)
        self.termin=False    
        self.move_done=False
        self.forward = n
        self.received_message = False
        self.hold_pose = rospy.Publisher('/cf1/hold_pos_cmd', Position, queue_size=2)
        super(Forward, self).__init__("Forward!")
        
    def pos_callback(self,msg):
        global drone_pos
        drone_pos=msg
        self.received_message = True

    def pub(self,goal):
        cmd = Position()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = goal.header.frame_id
        cmd.x = goal.pose.position.x
        cmd.y = goal.pose.position.y
        cmd.z = goal.pose.position.z
        _, _, yaw = euler_from_quaternion((goal.pose.orientation.x,
                                            goal.pose.orientation.y,
                                            goal.pose.orientation.z,
                                            goal.pose.orientation.w))
        cmd.yaw = math.degrees(yaw)
        self.hold_pose.publish(cmd)
        
    def update(self):
        global  drone_pos, goal
        if self.received_message == True:
            print('Forward: Message received')
            if self.move_done == False:   
                print('Forward: updating goal\n')
                goal.header.stamp = rospy.Time.now()
                goal.header.frame_id  =   drone_pos.header.frame_id
                goal.pose.position.y = drone_pos.pose.position.y
                goal.pose.position.x = drone_pos.pose.position.x + self.forward
                goal.pose.position.z = drone_pos.pose.position.z
                goal.pose.orientation= drone_pos.pose.orientation

                self.move_done=True
                self.pub(goal)
                self.forward = -1 * self.forward 
                return pt.common.Status.RUNNING
            else:
                return pt.common.Status.SUCCESS
        else:
                print('Forward: Message not receivedd yet')
                return pt.common.Status.FAILURE

    def terminate(self, new_status):
        if not self.termin == rst.reset:
            # Execution checker
            self.termin = not self.termin  
            self.move_done = False   
            self.received_message = False

# class Down(pt.behaviour.Behaviour):
        
#     def __init__(self,n):
#         # Create Publisher
#         self.pub_cmd = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
#         # Create Subscriber to keyboard
#         self.dr_pos=rospy.Subscriber('/cf1/pose', PoseStamped, self.pos_callback)
#         self.termin=False    
#         self.move_done=False
#         self.height=n
#         super(Down, self).__init__("Down!")
        
#     def pos_callback(self,msg):
#         global drone_pos
#         drone_pos=msg
        
#     def pub(self,goal):
#         cmd = Position()
#         cmd.header.stamp = rospy.Time.now()
#         cmd.header.frame_id = goal.header.frame_id
#         cmd.x = goal.pose.position.x
#         cmd.y = goal.pose.position.y
#         cmd.z = goal.pose.position.z
#         _, _, yaw = euler_from_quaternion((goal.pose.orientation.x,
#                                             goal.pose.orientation.y,
#                                             goal.pose.orientation.z,
#                                             goal.pose.orientation.w))
#         cmd.yaw = math.degrees(yaw)
#         self.pub_cmd.publish(cmd)
        
#     def update(self):
#         global  drone_pos,goal
#         if self.move_done==False:          
#             goal=PoseStamped()
#             goal.pose.position.y = drone_pos.pose.position.y
#             goal.pose.position.x = drone_pos.pose.position.x
#             goal.pose.position.z = drone_pos.pose.position.z-self.height
#             goal.pose.orientation= drone_pos.pose.orientation
#             self.move_done=True
#             self.pub(goal)
#             return pt.common.Status.RUNNING
#         else:
#             self.pub(goal)
#             return pt.common.Status.SUCCESS
#     def terminate(self, new_status):
        
#         if not self.termin == rst.reset:
#             # Execution checker
#             #print("Terminated Down movement ",reset)
#             rospy.loginfo("Resetting from bt_move.Down")
#             self.termin=not self.termin  
#             self.move_done=False
            
# class Right(pt.behaviour.Behaviour):
        
#     def __init__(self,n):
#         # Create Publisher
#         self.pub_cmd = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
#         # Create Subscriber to keyboard
#         self.dr_pos=rospy.Subscriber('/cf1/pose', PoseStamped, self.pos_callback)
#         self.termin=False    
#         self.move_done=False
#         self.right=n
#         super(Right, self).__init__("Right!")
        
#     def pos_callback(self,msg):
#         global drone_pos
#         drone_pos=msg
        
#     def pub(self,goal):
#         cmd = Position()
#         cmd.header.stamp = rospy.Time.now()
#         cmd.header.frame_id = goal.header.frame_id
#         cmd.x = goal.pose.position.x
#         cmd.y = goal.pose.position.y
#         cmd.z = goal.pose.position.z
#         _, _, yaw = euler_from_quaternion((goal.pose.orientation.x,
#                                             goal.pose.orientation.y,
#                                             goal.pose.orientation.z,
#                                             goal.pose.orientation.w))
#         cmd.yaw = math.degrees(yaw)
#         self.pub_cmd.publish(cmd)
        
#     def update(self):
#         global  drone_pos,goal
#         if self.move_done==False:          
#             goal=PoseStamped()
#             goal.pose.position.y = drone_pos.pose.position.y+self.right
#             goal.pose.position.x = drone_pos.pose.position.x
#             goal.pose.position.z = drone_pos.pose.position.z
#             goal.pose.orientation= drone_pos.pose.orientation
#             self.move_done=True
#             self.pub(goal)
#             return pt.common.Status.RUNNING
#         else:
#             self.pub(goal)
#             return pt.common.Status.SUCCESS
#     def terminate(self, new_status):
#         #global reset
#         #print("++++++++++++++++++++++++++++++++++ " , reset)

#         if not self.termin == rst.reset:
#             # Execution checker
#             #print("Terminated Right movement ",reset)
#             rospy.loginfo("Resetting from bt_move.Right")
#             self.termin=not self.termin  
#             self.move_done=False
            
# class Left(pt.behaviour.Behaviour):
        
#     def __init__(self,n):
#         # Create Publisher
#         self.pub_cmd = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
#         # Create Subscriber to keyboard
#         self.dr_pos=rospy.Subscriber('/cf1/pose', PoseStamped, self.pos_callback)
#         self.termin=False    
#         self.move_done=False
#         self.left=-n
#         super(Left, self).__init__("Left!")
        
#     def pos_callback(self,msg):
#         global drone_pos
#         drone_pos=msg
        
#     def pub(self,goal):
#         cmd = Position()
#         cmd.header.stamp = rospy.Time.now()
#         cmd.header.frame_id = goal.header.frame_id
#         cmd.x = goal.pose.position.x
#         cmd.y = goal.pose.position.y
#         cmd.z = goal.pose.position.z
#         _, _, yaw = euler_from_quaternion((goal.pose.orientation.x,
#                                             goal.pose.orientation.y,
#                                             goal.pose.orientation.z,
#                                             goal.pose.orientation.w))
#         cmd.yaw = math.degrees(yaw)
#         self.pub_cmd.publish(cmd)
        
#     def update(self):
#         global  drone_pos,goal
#         if self.move_done==False:          
#             goal=PoseStamped()
#             goal.pose.position.y = drone_pos.pose.position.y+self.left
#             goal.pose.position.x = drone_pos.pose.position.x
#             goal.pose.position.z = drone_pos.pose.position.z
#             goal.pose.orientation= drone_pos.pose.orientation
#             self.move_done=True
#             self.pub(goal)
#             return pt.common.Status.RUNNING
#         else:
#             self.pub(goal)
#             return pt.common.Status.SUCCESS
#     def terminate(self, new_status):
#         #global reset
#         #print("++++++++++++++++++++++++++++++++++ " , reset)

#         if not self.termin == rst.reset:
#             # Execution checker
#             #print("Terminated Left movement ",reset)
#             rospy.loginfo("Resetting from bt_move.Left")
#             self.termin=not self.termin  
#             self.move_done=False       

# class Backward(pt.behaviour.Behaviour):
        
    # def __init__(self,n):
    #     # Create Publisher
    #     self.pub_cmd = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
    #     # Create Subscriber to keyboard
    #     self.dr_pos=rospy.Subscriber('/cf1/pose', PoseStamped, self.pos_callback)
    #     self.termin=False    
    #     self.move_done=False
    #     self.backward=-n
    #     super(Backward, self).__init__("Backward!")
        
    # def pos_callback(self,msg):
    #     global drone_pos
    #     drone_pos=msg
        
    # def pub(self,goal):
    #     cmd = Position()
    #     cmd.header.stamp = rospy.Time.now()
    #     cmd.header.frame_id = goal.header.frame_id
    #     cmd.x = goal.pose.position.x
    #     cmd.y = goal.pose.position.y
    #     cmd.z = goal.pose.position.z
    #     _, _, yaw = euler_from_quaternion((goal.pose.orientation.x,
    #                                         goal.pose.orientation.y,
    #                                         goal.pose.orientation.z,
    #                                         goal.pose.orientation.w))
    #     cmd.yaw = math.degrees(yaw)
    #     self.pub_cmd.publish(cmd)
        
    # def update(self):
    #     global  drone_pos,goal
    #     if self.move_done==False:          
    #         goal=PoseStamped()
    #         goal.pose.position.x = drone_pos.pose.position.x+self.backward
    #         goal.pose.position.y = drone_pos.pose.position.y
    #         goal.pose.position.z = drone_pos.pose.position.z
    #         goal.pose.orientation= drone_pos.pose.orientation
    #         self.move_done=True
    #         self.pub(goal)
    #         return pt.common.Status.RUNNING
    #     else:
    #         self.pub(goal)
    #         return pt.common.Status.SUCCESS

    # def terminate(self, new_status):

    #     if not self.termin == rst.reset:
    #         # Execution checker
    #         print("Terminated Forward movement ",rst.reset)
    #         self.termin=not self.termin  
    #         self.move_done=False