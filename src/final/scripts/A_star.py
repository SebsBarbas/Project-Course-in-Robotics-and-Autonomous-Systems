#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Import relevant messages
from geometry_msgs.msg import PoseStamped
from crazyflie_driver.msg import Position
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Path
# Import Transforms and Frames 
from tf.transformations import *
import tf2_geometry_msgs
import tf2_ros
# Import relevant tools
import rospy
import math
import numpy as np
import os
import ast
from shapely.geometry import LineString, box, Polygon, Point


class pathPlanner():
    # NEVER CHANGES - static parameters, only run the first time bt is set up
    cell_constant = 0.2 # Making each cell in the grid the size of 0.2 [m]
    cell_x = 0
    cell_y = 0
    x_lb = 0
    x_ub = 0
    y_lb = 0
    y_ub = 0
    mapInfo = None
    obstacles = []
    walls = []
    obstacleInflation = 0.19
    wall_inflation = 0.05
    # Steps to enter new cell in grid
    step = 100   #change for max length of 
    delta_time = .01
    threshold = 0.05
    target_threshold = 0.24  #distance of target from target sign
    angle = np.linspace(0, 350, 10) # 10,360,36
    angle = np.radians(angle)

    # Create Publisher
    pathPub = rospy.Publisher('/move_base/TrajectoryPlanner', Path, queue_size=5)
    markerPub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=5)

    def __init__(self):
        # CHANGES - each time path planner is called
        self.grid = None
        self.xt = None
        self.yt = None
        self.yaw_t = None
        self.Start_node = None
        self.node = None
        self.hitta = None
        self.firstNode = True
        self.map_to_pathPlanner()

    def planPath(self, x0, y0, yaw0, signID):
               
        self.firstNode = True
        self.hitta = False
        self.signID = signID

        self.openSet = []
        self.closedSet = []

        #Target position    
        self.grid = [[None]*self.cell_x]*self.cell_y # Creating the matrix: [[empty]*x]*y

        targetSign = next((x for x in self.mapInfo["roadsigns"] if x["sign"] == self.signID), None)
        [xt0, yt0] = targetSign["pose"]["position"][:2]
        self.yaw_t = math.radians(targetSign["pose"]["orientation"][2]) + math.pi/2
        [self.xt, self.yt] = self.mapToGrid([xt0 + math.cos(self.yaw_t) * self.target_threshold, yt0 + math.sin(self.yaw_t) * self.target_threshold])

        self.Start_node = AstarNode(self, x0, y0, yaw0)
        self.openSet.append(self.Start_node)
        self.node = self.Start_node
        

        #Starting position
        [self.Start_node.x, self.Start_node.y] = self.mapToGrid([x0, y0])
        #print([self.Start_node.x, self.Start_node.y])
        self.grid[int(self.Start_node.y)][int(self.Start_node.x)] = self.openSet[0]

        tm_array = MarkerArray()

        #Creating marker for displaying start location
        tm = Marker()  
        tm.header.stamp = rospy.Time.now()
        tm.header.frame_id = "map"
        tm.id = 91
        tm.type = tm.CYLINDER
        tm.action = tm.ADD
        tm.pose.position.x = x0
        tm.pose.position.y = y0
        tm.pose.position.z = 0.4
        (tm.pose.orientation.x, tm.pose.orientation.y, tm.pose.orientation.z, tm.pose.orientation.w) = quaternion_from_euler(0, 0, 0)
        tm.scale.x = 0.05
        tm.scale.y = 0.05
        tm.scale.z = 0.8
        tm.color = ColorRGBA(0.2, 1, 1, 1)

        tm_array.markers.append(tm)

        #Creating marker for displaying target location
        tm = Marker()  
        tm.header.stamp = rospy.Time.now()
        tm.header.frame_id = "map"
        tm.id = 92
        tm.type = tm.CYLINDER
        tm.action = tm.ADD
        tm.pose.position.x = self.gridToMap([self.xt, self.yt])[0]
        tm.pose.position.y = self.gridToMap([self.xt, self.yt])[1]
        tm.pose.position.z = 0.4
        (tm.pose.orientation.x, tm.pose.orientation.y, tm.pose.orientation.z, tm.pose.orientation.w) = quaternion_from_euler(0, 0, 0)
        tm.scale.x = 0.05
        tm.scale.y = 0.05
        tm.scale.z = 0.8
        tm.color = ColorRGBA(1, 1, 0, 1)

        tm_array.markers.append(tm)

        #Creating marker for displaying target roadsign
        tm = Marker()  
        tm.header.stamp = rospy.Time.now()
        tm.header.frame_id = "map"
        tm.id = 93
        tm.type = tm.CUBE
        tm.action = tm.ADD
        tm.pose.position.x = [xt0 + math.cos(self.yaw_t) * 0.02, yt0 + math.sin(self.yaw_t) * 0.02][0]
        tm.pose.position.y = [xt0 + math.cos(self.yaw_t) * 0.02, yt0 + math.sin(self.yaw_t) * 0.02][1]
        tm.pose.position.z = 0.4
        (tm.pose.orientation.x, tm.pose.orientation.y, tm.pose.orientation.z, tm.pose.orientation.w) = quaternion_from_euler(0, 0, self.yaw_t-math.pi/2)
        tm.scale.x = 0.2
        tm.scale.y = 0.01
        tm.scale.z = 0.2
        tm.color = ColorRGBA(1, 0, 0, 1)

        tm_array.markers.append(tm)

        
        rospy.sleep(0.1)
        self.markerPub.publish(tm_array)

        self.Create_Node()
        result = self.PathPlanning()
        # print(result)
        return result

    def map_to_pathPlanner(self):
        #Read map info from json file
        mapFilePath = "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/src/course_packages/dd2419_resources/worlds_json/awesome.world.json"
        mapString = ""

        with open(mapFilePath, "r") as file:
            for line in file:                           #for each row
                l = line.strip().replace(" ", "")                        #remove all blankspace
                mapString += l

        self.mapInfo = ast.literal_eval(mapString)       #convert string representation of read file into dictionary through some kind of black magic

        # Upper bounds
        self.x_ub = self.mapInfo["airspace"]["max"][0] +  6 * self.threshold # From json
        self.y_ub = self.mapInfo["airspace"]["max"][1] +  6 * self.threshold # from json
        # Lower bounds
        self.x_lb = self.mapInfo["airspace"]["min"][0] -  6 * self.threshold # From json
        self.y_lb = self.mapInfo["airspace"]["min"][1] -  6 * self.threshold # from json

        # Making the grid
        self.cell_x = int((self.x_ub - self.x_lb) / self.cell_constant) # Number of cells in x
        self.cell_y = int((self.y_ub - self.y_lb) / self.cell_constant) # Number of cells in y


        lines = []
        
        for wall in self.mapInfo["walls"]:
            wallStart = [wall["plane"]["start"][0], wall["plane"]["start"][1], wall["plane"]["start"][2]]
            wallStop = [wall["plane"]["stop"][0], wall["plane"]["stop"][1], wall["plane"]["start"][2]]
            lines.append(LineString([(wallStart[0], wallStart[1]), (wallStop[0], wallStop[1])]))

        for line in lines:
            self.obstacles.append(line.buffer(self.obstacleInflation))
            self.walls.append(line.buffer(self.wall_inflation))

        tm_array = MarkerArray()


        #Creating marker for displaying airspace boundaries
        airspaceStart = self.mapInfo["airspace"]["min"]
        airspaceStop = self.mapInfo["airspace"]["max"]

        tm = Marker()            
        tm.header.stamp = rospy.Time.now()
        tm.header.frame_id = "map"
        tm.id = 90
        tm.type = tm.CUBE
        tm.action = tm.ADD
        tm.pose.position.x = (airspaceStart[0] + airspaceStop[0])/2
        tm.pose.position.y = (airspaceStart[1] + airspaceStop[1])/2
        tm.pose.position.z = 0.01
        (tm.pose.orientation.x, tm.pose.orientation.y, tm.pose.orientation.z, tm.pose.orientation.w) = quaternion_from_euler(0, 0, 0)
        tm.scale.x = abs(airspaceStart[0]) + abs(airspaceStop[0])
        tm.scale.y = abs(airspaceStart[1]) + abs(airspaceStop[1])
        tm.scale.z = 0.02

        tm.color = ColorRGBA(1, 1, 1, 0.3)

        tm_array.markers.append(tm)

        for i, ii in enumerate(lines):

            #Creating marker for displaying wall
            tm = Marker()
            wall_angle = math.atan2((list(ii.coords)[0][1] - list(ii.coords)[1][1]), (list(ii.coords)[0][0] - list(ii.coords)[1][0]))            
            tm.header.stamp = rospy.Time.now()
            tm.header.frame_id = "map"
            tm.id = 100+i*5
            tm.type = tm.CUBE
            tm.action = tm.ADD
            tm.pose.position.x = (list(ii.coords)[0][0] + list(ii.coords)[1][0])/2
            tm.pose.position.y = (list(ii.coords)[0][1] + list(ii.coords)[1][1])/2
            tm.pose.position.z = 1.5
            (tm.pose.orientation.x, tm.pose.orientation.y, tm.pose.orientation.z, tm.pose.orientation.w) = quaternion_from_euler(0, 0, wall_angle)
            tm.scale.x = math.sqrt((list(ii.coords)[0][0] - list(ii.coords)[1][0])**2 + (list(ii.coords)[0][1] - list(ii.coords)[1][1])**2)
            tm.scale.y = 0.02
            tm.scale.z = 3
            tm.color = ColorRGBA(1, .5, .1, 1)
            tm_array.markers.append(tm)

            #Creating markers for displaying inflated wall
            tm = Marker()
            wall_angle = math.atan2((list(ii.coords)[0][1] - list(ii.coords)[1][1]), (list(ii.coords)[0][0] - list(ii.coords)[1][0]))            
            tm.header.stamp = rospy.Time.now()
            tm.header.frame_id = "map"
            tm.id = 101+i*5
            tm.type = tm.CUBE
            tm.action = tm.ADD
            tm.pose.position.x = (list(ii.coords)[0][0] + list(ii.coords)[1][0])/2
            tm.pose.position.y = (list(ii.coords)[0][1] + list(ii.coords)[1][1])/2
            tm.pose.position.z = 1.5
            (tm.pose.orientation.x, tm.pose.orientation.y, tm.pose.orientation.z, tm.pose.orientation.w) = quaternion_from_euler(0, 0, wall_angle)
            tm.scale.x = math.sqrt((list(ii.coords)[0][0] - list(ii.coords)[1][0])**2 + (list(ii.coords)[0][1] - list(ii.coords)[1][1])**2)
            tm.scale.y = self.obstacleInflation*2
            tm.scale.z = 3
            tm.color = ColorRGBA(1, .5, .1, 0.2)
            tm_array.markers.append(tm)

            tm = Marker()
            wall_angle = math.atan2((list(ii.coords)[0][1] - list(ii.coords)[1][1]), (list(ii.coords)[0][0] - list(ii.coords)[1][0]))            
            tm.header.stamp = rospy.Time.now()
            tm.header.frame_id = "map"
            tm.id = 102+i*5
            tm.type = tm.CYLINDER
            tm.action = tm.ADD
            tm.pose.position.x = list(ii.coords)[0][0]
            tm.pose.position.y = list(ii.coords)[0][1]
            tm.pose.position.z = 1.5
            (tm.pose.orientation.x, tm.pose.orientation.y, tm.pose.orientation.z, tm.pose.orientation.w) = quaternion_from_euler(0, 0, wall_angle)
            tm.scale.x = self.obstacleInflation*2
            tm.scale.y = self.obstacleInflation*2
            tm.scale.z = 3
            tm.color = ColorRGBA(1, .5, .1, 0.2)
            tm_array.markers.append(tm)

            tm = Marker()
            wall_angle = math.atan2((list(ii.coords)[0][1] - list(ii.coords)[1][1]), (list(ii.coords)[0][0] - list(ii.coords)[1][0]))            
            tm.header.stamp = rospy.Time.now()
            tm.header.frame_id = "map"
            tm.id = 103+i*5
            tm.type = tm.CYLINDER
            tm.action = tm.ADD
            tm.pose.position.x = list(ii.coords)[1][0]
            tm.pose.position.y = list(ii.coords)[1][1]
            tm.pose.position.z = 1.5
            (tm.pose.orientation.x, tm.pose.orientation.y, tm.pose.orientation.z, tm.pose.orientation.w) = quaternion_from_euler(0, 0, wall_angle)
            tm.scale.x = self.obstacleInflation*2
            tm.scale.y = self.obstacleInflation*2
            tm.scale.z = 3
            tm.color = ColorRGBA(1, .5, .1, 0.2)
            tm_array.markers.append(tm)
        

        rospy.sleep(.5)
        self.markerPub.publish(tm_array)

    def obstacle_avoidance(self, x,y):
        [checkX, checkY] = self.gridToMap([x, y])

        # if firstNode:
        #     for w in self.walls:
        #         if w.contains(Point(checkX, checkY)):
        #             return True
        # else:   
        for c in self.obstacles:
            if c.contains(Point(checkX, checkY)):
                return True

        if (x < 0) or (x > self.cell_x) or (y < 0) or (y > self.cell_y):  
            return True 

    def stepp(self, x, y, yaw, path):
        # state rate
        dx     = math.cos(yaw)
        dy     = math.sin(yaw)
        dt_yaw = math.tan(path)
        # new state (forward Euler integration)
        xn     = x    + self.delta_time * dx
        yn     = y    + self.delta_time * dy
        yaw_n  = yaw  + self.delta_time * dt_yaw
        # yaw_n = path

        return xn, yn, yaw_n

    def Create_Node(self):
        while (len(self.openSet) > 0):
            if len(self.node.children) == 0:
                for path in self.angle:
                    path = path+self.node.yaw
                    xn, yn, yaw_n = self.stepp(self.node.x,self.node.y,self.node.yaw,path)
                    count = 0
                    try: 
                        while self.grid[int(yn)][int(xn)] != None and count < self.step:
                            xn, yn, yaw_n = self.stepp(xn,yn,yaw_n,path)
                            count += 1
                            obs = self.obstacle_avoidance(xn,yn)
                            if obs:
                                break
                            if (abs(xn - self.xt) < self.threshold) and (abs(yn - self.yt) < self.threshold): # Node = Target
                                #Make final node point toward target roadsign
                                NewNode = AstarNode(self, self.xt, self.yt, self.yaw_t - math.pi)
                                NewNode.g = self.node.g + count * self.delta_time
                                NewNode.f = NewNode.g + NewNode.h
                                NewNode.parent = self.node
                                self.node = NewNode
                                self.hitta = True
                                return
                            
                        NewNode = AstarNode(self, xn, yn, yaw_n)
                        obs = self.obstacle_avoidance(NewNode.x, NewNode.y)
                        # Tests whether a point is within boundaries and obstacle
                        if obs:
                            self.closedSet.append(NewNode)
                        else:
                            # self.count_ += 1
                            # if self.grid[int(yn)][int(xn)] != None and self.firstnode and self.count_ > 2: # Only enter after stepping into the first cell from starting position
                            #     self.firstnode = False
                            # Make the new node a part of the children
                            # Creating new node, with the old node as a parent
                            NewNode.g = self.node.g + count * self.delta_time
                            NewNode.f = NewNode.g + NewNode.h
                            NewNode.parent = self.node
                            self.node.children.append(NewNode)
                            self.openSet.append(NewNode)
                            if self.grid[int(yn)][int(xn)] == None:
                                self.grid[int(yn)][int(xn)] = NewNode
                            else:
                                if NewNode.f < self.grid[int(yn)][int(xn)].f:
                                    self.grid[int(yn)][int(xn)] = NewNode
                    except Exception as e:
                        #print("Error in path planning: " + str(e))
                        pass
            # Take the best node away from open and put it in closed
            self.openSet.sort(key = lambda x: x.f)
            self.node = self.openSet.pop(0)
            self.closedSet.append(self.node)

    def PathPlanning(self):
        pub_node = []
        while not self.node.parent == None:
            pub_node.append(self.node)
            self.node = self.node.parent
        pub_node.append(self.Start_node)
        pub_node.reverse()

        pathMsg = Path()
        pathMsg.header.stamp = rospy.Time.now()
        pathMsg.header.frame_id = "map"

        if self.hitta:
            for ii in pub_node:
                pos_ = PoseStamped()
                pos_.header.stamp = rospy.Time.now()
                pos_.header.frame_id = "map"
                pos_.pose.position.z = 0.35
                q_new = quaternion_from_euler(0.0 ,0.0 , ii.yaw) 
                pos_.pose.orientation.x, pos_.pose.orientation.y, pos_.pose.orientation.z, pos_.pose.orientation.w = (q_new[0],q_new[1],q_new[2],q_new[3])
                [pos_.pose.position.x, pos_.pose.position.y] = self.gridToMap([ii.x, ii.y])
                pathMsg.poses.append(pos_)

            rospy.sleep(0.1)
            pathMsg.poses.pop(0)
            self.pathPub.publish(pathMsg)
        
        return pathMsg
        
        
    def mapToGrid(self, mapCoords):       #translates from real coords to grid coords
        aMatrix = np.array([[self.cell_x/(self.x_ub - self.x_lb), 0], [0, self.cell_y/(self.y_ub - self.y_lb)]])
        bMatrix = np.array([self.x_lb, self.y_lb])
        mMatrix = np.array([mapCoords[0], mapCoords[1]])
        gMatrix = np.matmul(aMatrix, np.subtract(mMatrix, bMatrix))
        return(gMatrix.tolist())


    def gridToMap(self, gridCoords):      #translates from grid coords to real coords
        aMatrix = np.array([[float(self.x_ub - self.x_lb)/float(self.cell_x), 0.0], [0.0, float(self.y_ub - self.y_lb)/float(self.cell_y)]])
        bMatrix = np.array([float(self.x_lb), float(self.y_lb)])
        gMatrix = np.array([float(gridCoords[0]), float(gridCoords[1])])
        mMatrix = np.add(np.matmul(aMatrix, gMatrix), bMatrix)
        return(mMatrix.tolist())

class AstarNode(pathPlanner):    
    def __init__(self, path_planner, x, y, path):
        self.x = x
        self.y = y
        self.yaw = path
        self.children = []
        self.parent = None
        self.g = 0 # Actual cost to next step
        self.h = abs(self.x - path_planner.xt) + abs(self.y - path_planner.yt) # f = g + h    
        self.f = self.g + self.h


def main():
    rospy.init_node('A_star')
    pp = pathPlanner()

if __name__== '__main__':
    main()