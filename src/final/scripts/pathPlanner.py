#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib

from project.msg import pathPlanningAction, pathPlanningActionFeedback
import A_star

class pathPlanner(object):
    # create messages that are used to publish feedback/result
    _feedback = String()   #should be fetched from pathPlanning.action instead
    _result = Path()       #should be fetched from pathPlanning.action instead

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, pathPlanningAction, execute_cb=self.execute_cb, auto_start = False) 
        self._as.start()
      
    def execute_cb(self, goal): 
        # helper variables
        r = rospy.Rate(1)       

        
        pp = Astar()
        pp.mapToAstar(x0, y0, path0)
        path = pp.pathPlanning(pp.createNodes())
          
        if success:
            self._result.sequence = self._feedback.sequence
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('pathPlanner_server')
    server = pathPlanner(rospy.get_name())
    rospy.spin()