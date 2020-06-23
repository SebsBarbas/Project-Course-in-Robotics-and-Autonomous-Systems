#!/usr/bin/env python

from milestone2.srv import pathPlanning
from A_star import pathPlanner, AstarNode
import rospy

pp = None

def cb(req):
    rospy.loginfo("ppServer received request")
    path = pp.planPath(req.x0, req.y0, req.yaw0, req.targetSign)
    rospy.loginfo("ppServer handled request, returning path of length " + str(len(path.poses)) + " to ppClient")
    return path 

def ppServer():
    global pp
    rospy.init_node('ppServer')
    pp = pathPlanner()
    s = rospy.Service('path_planning', pathPlanning, cb)
    rospy.loginfo("Path planning server set up")
    rospy.spin()

if __name__ == "__main__":
    ppServer()