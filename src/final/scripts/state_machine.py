#!/usr/bin/env python
from __future__ import print_function
import rospy
import math
import sys 
import tf
import roslib
import cv2 
import numpy as np
from geometry_msgs.msg import PoseStamped
from crazyflie_driver.msg import Position
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
import copy

# OUR FILES
from A_star import pathPlanner, AstarNode
from milestone2.srv import pathPlanning


def actual_pose_callback(msg):
    global drone_position
    drone_position = msg


def drone_up():
    global goal_position, move_up_not_done
    #First, we are going to move up
    goal_position.header.stamp = rospy.Time.now()
    goal_position.header.frame_id = drone_position.header.frame_id
    goal_position.pose.position.x = drone_position.pose.position.x
    goal_position.pose.position.y = drone_position.pose.position.y
    goal_position.pose.position.z = 0.35
    goal_position.pose.orientation = drone_position.pose.orientation
    hold_pos.publish(goal_position)    
    move_up_not_done = False


def spin_drone(deg_):
    global goal_position
    rospy.loginfo("Beginning spin")
    angle = np.linspace(0, deg_, 9)[1:]
    angle = np.radians(angle)
    goal_position.pose.position.x = drone_position.pose.position.x
    goal_position.pose.position.y = drone_position.pose.position.y
    q_orig = [drone_position.pose.orientation.x, drone_position.pose.orientation.y, drone_position.pose.orientation.z, drone_position.pose.orientation.w]
    rospy.sleep(5)

    for yaw in angle:
        q_rot = quaternion_from_euler(0.0, 0.0, yaw)
        q_new = quaternion_multiply(q_rot, q_orig)
        goal_position.pose.orientation.x, goal_position.pose.orientation.y, goal_position.pose.orientation.z, goal_position.pose.orientation.w = (q_new[0],q_new[1],q_new[2],q_new[3])
        hold_pos.publish(goal_position)
        t_first = rospy.get_time()
        rospy.sleep(1) 
        t_last = rospy.get_time()
        print("time for sleep in spin: " + str(t_last - t_first))

    rospy.sleep(5)
    rospy.loginfo("Spinning done")
    return False


def plan():
    global path, goal_position, t
    x0 = float(drone_position.pose.position.x)
    y0 = float(drone_position.pose.position.y)
    _, _, yaw = euler_from_quaternion([drone_position.pose.orientation.x,
                drone_position.pose.orientation.y,
                drone_position.pose.orientation.z,
                drone_position.pose.orientation.w])

    yaw0 = float(yaw)
    rospy.loginfo("ppClient requesting path from: x = " + str(round(x0, 2)) + ", y = " + str(round(y0, 2)) + " and yaw = " + str(round(yaw0, 2)) + " degrees, to traffic sign: " + str(signs[0]))
    path = proxy(x0, y0, yaw0, signs[0]).plannedPath.poses

    if len(path) > 0:
        rospy.loginfo("ppClient received valid path")
        goal_position = path.pop(0)
        hold_pos.publish(goal_position)        
        #t = np.array([goal_position.pose.position.x - drone_position.pose.position.x, goal_position.pose.position.y - drone_position.pose.position.y])   #store current part of the path as vector
        return False
    else:
          rospy.loginfo("ppClient received invalid path, requesting new path")
          return True


def path_follow():
    global path, goal_position, t
    print("in path follow")
    if len(path) != 0:
        #prev_position = copy.deepcopy(goal_position)
        goal_position = path.pop(0)
        hold_pos.publish(goal_position)
        #t = np.array([goal_position.pose.position.x - prev_position.pose.position.x, goal_position.pose.position.y - prev_position.pose.position.y])   #store current part of the path as vector
        rospy.sleep(1)
        rospy.loginfo("path follower running")
        return True
    if len(path) == 0: 
        return False
    

def in_position():
    if flag_up:
        hight = goal_position.pose.position.z - drone_position.pose.position.z
        if hight < 0.01:
            return True   
        else:
            return False

    dist = np.sqrt((goal_position.pose.position.x - drone_position.pose.position.x)**2 + (goal_position.pose.position.y - drone_position.pose.position.y)**2)
    if dist < 0.08:
        return True
    else:
        return False 


    # FLAGS

flag_up = True
move_up_not_done = True
flag_spin_up = False
flag_spin = False
flag_planner = False
flag_path = False

# SERVICES
rospy.wait_for_service('path_planning')
proxy = rospy.ServiceProxy('path_planning', pathPlanning)

# SUBSCRIBERS
dr_pos=rospy.Subscriber('/cf1/pose', PoseStamped, actual_pose_callback)

# PUBLISHERS
#hold_pose = rospy.Publisher('/cf1/hold_pos_cmd', Position, queue_size=2)
hold_pos = rospy.Publisher('/cf1/hold_pos', PoseStamped, queue_size=2)

# GLOBAL
signs = ["no_bicycle", "residential"]
drone_position = PoseStamped()
goal_position = PoseStamped()
t = None #np.array() object used for representing the current stretch of the path 
path = None
# pos_thresh = 0.03


def main():
    global drone_position, goal_position, flag_up, flag_spin_up, flag_planner, flag_path, flag_spin, signs, move_up_not_done
    rospy.init_node("state_machine")
    rate = rospy.Rate(10)
    goal_position.header.frame_id = "cf1/odom"
    # IMPORTING THE 
    # drone_up()

    while not rospy.is_shutdown():
        # ------------- Init part ---------------
        if flag_up:
            print("flag_up ", move_up_not_done)
            if move_up_not_done:
                drone_up()
            if in_position():
                flag_up = False
                flag_spin_up = True

        # if flag_spin_up:
        #     print("flag_spin_up")
        #     flag_spin_up = spin_drone(360) # Stuck in while until done spinning
        #     flag_planner = True



        # # ------------- Recurring part ---------------
        # if flag_planner:
        #     print("flag_planner")
        #     flag_planner = plan()
        #     if not flag_planner:
        #         flag_path = True
                
                
            
        # if flag_path:
        #     print("flag_path")
        #     #p = np.array([goal_position.pose.position.x - drone_position.pose.position.x, goal_position.pose.position.y - drone_position.pose.position.y)
        #     #alpha = math.acos(np.dot(p, t)) / (np.linalg.norm(t) * np.linalg.norm(p))     #angle between path and current position
        #     # deviation = np.sin(alpha) * np.linalg.norm(p)

        #     #if deviation > some threshold i dont know: #Go back to planning stage
        #         #rospy.loginfo("Deviated too far from original path, re-planning")
        #         #flag_path = False
        #         #flag_planner = True

        #     #maybe the stuff above should be checked after in_position is checked


        #     #else:
        #     if in_position():   
        #         flag_path = path_follow()
        #         if not flag_path:
        #             flag_spin = True

        # if flag_spin:
        #     print("flag_spin")
        #     flag_spin = spin_drone(360) # Stuck in while until done spinning
        #     flag_planner = True
        #     signs.append(signs.pop(0))

        rate.sleep()



if __name__ == '__main__':
    main()