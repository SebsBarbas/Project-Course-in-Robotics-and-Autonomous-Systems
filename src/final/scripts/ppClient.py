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
from crazyflie_driver.msg import Position
from milestone2.srv import pathPlanning
from geometry_msgs.msg import PoseStamped
from A_star import pathPlanner, AstarNode
from ttictoc import TicToc
import Reset as rst

# Global stuff
drone_pos=Position()

class Planner(pt.behaviour.Behaviour):
  def __init__(self):
    self.map_pos=rospy.Subscriber('/cf1/pose_map', Position, self.callback)
    self.pub_hold_pos  = rospy.Publisher('/cf1/hold_pos', PoseStamped, queue_size=2)
    rospy.loginfo("ppClient waiting for service")
    rospy.wait_for_service('path_planning')
    self.proxy = rospy.ServiceProxy('path_planning', pathPlanning)
    rospy.loginfo("ppClient finished waiting for service")
    self.signs = { 1: 'airport', 2: 'dangerous_curve_left', 3: 'dangerous_curve_right', 4: 'follow_left',
                  5: 'follow_right', 6: 'junction', 7: 'no_bicycle', 8: 'no_heavy_truck', 9: 'no_parking',
                  10: 'no_stopping_and_parking', 11: 'residential', 12: 'road_narrows_from_left', 13: 'road_narrows_from_right',
                  14: 'roundabout_warning', 15: 'stop' }

    self.cmd = Position()
    self.termin=False
    self.plan=False
    self.callback_called = False
    super(Planner, self).__init__("Planner!")

  def callback(self,data):
    #   For any method that requires it (subscriptros).
    global drone_pos
    drone_pos = data
    self.callback_called = True
      
  def update(self):
    #Update Stuff
    if not rst.nextSign:
      self.route = self.signs[11]
    if rst.nextSign:
      self.route = self.signs[7]

    if not self.plan and self.callback_called == True:
      x0 = float(drone_pos.x)
      y0 = float(drone_pos.y)
      yaw0 = float(drone_pos.yaw) # This is in degrees
      rospy.loginfo("ppClient requesting path from: x = " + str(round(x0, 2)) + ", y = " + str(round(y0, 2)) + " and yaw = " + str(round(yaw0, 2)) + " degrees, to traffic sign: " + str(self.route))
      #rospy.loginfo("requesting path")
      path = self.proxy(x0, y0, yaw0, self.route).plannedPath
      # rospy.loginfo("ppClient received path of length " + str(len(path.poses)))

      if len(path.poses) > 0:
        rospy.loginfo("ppClient received valid path")
        self.first_pose=False
        self.plan=True
        return pt.common.Status.SUCCESS
      else:
          rospy.loginfo("ppClient received invalid path, requesting new path")
          return pt.common.Status.RUNNING
      
    else:
      return pt.common.Status.SUCCESS

  def terminate(self, new_status):
    if not self.termin == rst.reset:
        rospy.loginfo("Resetting from ppClient")
        # Execution checker
        self.termin=not self.termin  
        self.plan=False
        self.callback_called = False