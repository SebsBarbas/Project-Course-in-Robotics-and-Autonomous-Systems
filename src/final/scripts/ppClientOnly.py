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

from milestone2.srv import pathPlanning
from A_star import *
from ttictoc import TicToc

reset=False
drone_pos=PoseStamped()


class Planner(pt.behaviour.Behaviour):

  def __init__(self):
    self.termin=False
    self.plan=False
    self.dr_pos=rospy.Subscriber('/cf1/pose', PoseStamped, self.callback)
    self.pub_hold_pos  = rospy.Publisher('/cf1/hold_pos', PoseStamped, queue_size=2)
    # self.pathPub = rospy.Publisher('/move_base/TrajectoryPlanner', Path, queue_size=5)
    rospy.loginfo("ppClient waiting for service")
    rospy.wait_for_service('path_planning')
    self.proxy = rospy.ServiceProxy('path_planning', pathPlanning)
    rospy.loginfo("ppClient finished waiting for service")
    self.signs = { 1: 'airport', 2: 'dangerous_curve_left', 3: 'dangerous_curve_right', 4: 'follow_left',
                   5: 'follow_right', 6: 'junction', 7: 'no_bicycle', 8: 'no_heavy_truck', 9: 'no_parking',
                   10: 'no_stopping_and_parking', 11: 'residential', 12: 'road_narrows_from_left', 13: 'road_narrows_from_right',
                   14: 'roundabout_warning', 15: 'stop' }
    self.route = [self.signs[15]]
    self.cmd = Position()
    super(Planner, self).__init__("Planner!")

  def callback(self,data):
    #   For any method that requires it (subscriptros).
    global drone_pos
    drone_pos = data
      
  def update(self):
    
    #Update Stuff
    if not self.plan:
      x0 = drone_pos.pose.position.x
      y0 = drone_pos.pose.position.y
      _, _, yaw0 = euler_from_quaternion((drone_pos.pose.orientation.x,
                                                  drone_pos.pose.orientation.y,
                                                  drone_pos.pose.orientation.z,
                                                  drone_pos.pose.orientation.w))
      t = TicToc()
      t.tic()
      path = self.proxy(x0, y0, yaw0, self.route[0]).plannedPath
      t.toc()
      print(t.elapsed)

      if len(path.poses) > 0:
        rospy.loginfo("pp success")
        self.first_pose=False
        self.plan=True
        print('We have sent the position!')
        return pt.common.Status.SUCCESS
    # if hitta == False:
      else:
        rospy.loginfo("pp failure")
        return pt.common.Status.FAILURE
      
    else:
      #rospy.loginfo("pp running")
      return pt.common.Status.SUCCESS

  def terminate(self, new_status):
    global reset

    if not self.termin == reset:
        rospy.loginfo("planner reset")
        # Execution checker
        self.termin=not self.termin  
        self.stereo_done=False