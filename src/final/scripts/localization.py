#!/usr/bin/env python

import rospy, tf2_ros
from geometry_msgs.msg import TransformStamped, Vector3,Pose,PoseStamped, Quaternion
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from tf.transformations import *
from aruco_msgs.msg import MarkerArray
from std_msgs.msg import Bool
import math, statistics
import tf_conversions
import tf
import cv2
import os
import ast
import numpy as np
import copy
from  scipy.signal import butter,lfilter,freqz

import pprint
# Global stuff
markers = None
savedDetections = {} 
estimated_poses={}
dict_ekf ={}
json_markers = {}
estimated_tf={}
savedtf={}
init=True
localize=Bool()
localize.data = False
offset_hist=[]
pp=pprint.PrettyPrinter(indent=4)
order = 5
cutoff = 1.0
fs = 3     # sample rate, Hz


def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a


def butter_lowpass_filter(data, cutoff, fs, order=5):
    print(data)
    #b, a = butter_lowpass(cutoff, fs, order=order)
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = lfilter(b, a, [data])
    return y

def markers_callback(msg):
    global markers, savedDetections, estimated_poses, dict_ekf
    markers = msg.markers
     # Dictionary for Kalman filter (we will have 3 aruco markers for the basic case)
    estimated_dict = {}
    for i in markers:
        if str(i.id) not in savedDetections.keys():
            savedDetections[str(i.id)]=[(msg.header.stamp.secs, i.pose.pose)]
            estimated_poses[str(i.id)] = PoseStamped()
            #Making sure that a Kalman filter is initialized for each marker if we don't have the Aruco Marker registered in the dictionary.
            dict_ekf[str(i.id)]=cv2.KalmanFilter(7,7) # Initialization of kalman filter.
            trans_mat = np.identity(7, np.float32)
            dict_ekf[str(i.id)].transitionMatrix = trans_mat          #x'=Ax+BU         transition matrix is A  
            dict_ekf[str(i.id)].measurementMatrix = trans_mat 
            dict_ekf[str(i.id)].processNoiseCov = cv2.setIdentity(dict_ekf[str(i.id)].processNoiseCov, 1e3) #4     
            dict_ekf[str(i.id)].measurementNoiseCov = cv2.setIdentity(dict_ekf[str(i.id)].measurementNoiseCov,1)#2   
            dict_ekf[str(i.id)].errorCovPost = cv2.setIdentity(dict_ekf[str(i.id)].errorCovPost)#, 1) 


        else:
            savedDetections[str(i.id)]=[(msg.header.stamp.secs, i.pose.pose)]
            # Once the filter is initialized, for all new iterations:
            translation = np.array([[i.pose.pose.position.x,i.pose.pose.position.y,i.pose.pose.position.z]], np.float32).reshape(-1, 1)
            rot_ang = np.array([i.pose.pose.orientation.x, i.pose.pose.orientation.y, i.pose.pose.orientation.z, i.pose.pose.orientation.w], np.float32).reshape(-1, 1)
                                        #    x,y,z       roll,pitch,yaw
            measurements = np.concatenate((translation, rot_ang))
            prediction = dict_ekf[str(i.id)].predict()
            estimated =dict_ekf[str(i.id)].correct(measurements.reshape(-1,1))
            estimated_poses[str(i.id)].header.stamp = msg.header.stamp
            estimated_poses[str(i.id)].header.frame_id=i.id
            estimated_poses[str(i.id)].pose.position.x = estimated[0]#i.pose.pose.position.x
            estimated_poses[str(i.id)].pose.position.y = estimated[1]#i.pose.pose.position.y
            estimated_poses[str(i.id)].pose.position.z = estimated[2]#i.pose.pose.position.z
            estimated_poses[str(i.id)].pose.orientation.x = estimated[3]#i.pose.pose.orientation.x
            estimated_poses[str(i.id)].pose.orientation.y = estimated[4]#i.pose.pose.orientation.y
            estimated_poses[str(i.id)].pose.orientation.z = estimated[5]#i.pose.pose.orientation.z
            if isinstance(i.pose.pose.orientation.w, list):
                estimated_poses[str(i.id)].pose.orientation.w = i.pose.pose.orientation.w[0]
            else:
                estimated_poses[str(i.id)].pose.orientation.w = i.pose.pose.orientation.w  

def greta(m):
    global savedDetections
    
    if m.header.frame_id!='' and (rospy.Time.now().secs - savedDetections[str(m.header.frame_id)][0][0] < 1):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'cf1/camera_link'
        t.child_frame_id = 'aruco/estimated_' + str(m.header.frame_id)

        t.transform.translation = Vector3(*[m.pose.position.x, m.pose.position.y, m.pose.position.z])
        [t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w] = (m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w)
        n = np.linalg.norm([t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w])
        t.transform.rotation.x /= n
        t.transform.rotation.y /= n
        t.transform.rotation.z /= n
        t.transform.rotation.w /= n
        return t
    else:
        return 0

def Map2localization(trans):
    global estimated_tf, init, localize, offset_hist,map2odom
    offset=TransformStamped()
    # We want to transform the detected aruco pose to map frame to do a reasonable comparison
    qinv=quaternion_from_euler(0, 0, 0)
    qdetected=quaternion_from_euler(0,0,0)
    if init:
         #Making sure that a Kalman filter is initialized for each marker if we don't have the Aruco Marker registered in the dictionary.
        estimated_tf=cv2.KalmanFilter(6,6) # Initialization of kalman filter.
        trans_mat = np.identity(6, np.float32)
        estimated_tf.transitionMatrix = trans_mat          #x'=Ax+BU         transition matrix is A  
        estimated_tf.measurementMatrix = trans_mat 
        estimated_tf.processNoiseCov = cv2.setIdentity(estimated_tf.processNoiseCov, 1e-3) #4     
        estimated_tf.measurementNoiseCov = cv2.setIdentity(estimated_tf.measurementNoiseCov,1e-2)#2   
        estimated_tf.errorCovPost = cv2.setIdentity(estimated_tf.errorCovPost)#, 1) 
        init=False
        

    if not buff.can_transform(trans.child_frame_id, 'cf1/odom', trans.header.stamp, rospy.Duration(0.5)):
        rospy.logwarn_throttle(5.0 , 'No tranform from %s to map' % trans.child_frame_id)
        return
    else:
        t=TransformStamped()
        try:
            # We want to keep the relative position.... We can calculate the error betwen these frames.
            t = buff.lookup_transform('cf1/odom',trans.child_frame_id, rospy.Time(0),rospy.Duration(0.5))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn_throttle(5.0 , 'No tranform from %s to odom' % trans.child_frame_id)

            return           
        
        #t.transform.translation.z=0.35
        #==============================================================================
        #Ludvig's solution
        #==============================================================================
        Fodom = tf_conversions.fromTf(        
            ((t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z),
            (t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w))
        )
        Fmap  = tf_conversions.fromMsg(json_markers[int(trans.child_frame_id[-1])])

        Fdiff = Fmap * Fodom.Inverse()
        

        offset = TransformStamped()
        offset.header.stamp = rospy.Time.now()
        offset.header.frame_id = 'map'
        offset.child_frame_id =  'cf1/odom'
        ((offset.transform.translation.x,
            offset.transform.translation.y,
            offset.transform.translation.z),
            (offset.transform.rotation.x,
            offset.transform.rotation.y,
            offset.transform.rotation.z,
            offset.transform.rotation.w)) = tf_conversions.toTf(Fdiff)
        #======================================
        #======================================
        #=====================================
        #offset.transform.translation.z = 0 # Not considering any changes in z-axis
        #print("before filter " + str(offset.transform.translation.x) + " " + str(offset.transform.translation.y) + " " + str(offset.transform.translation.z))
        """
        print("")
        offset.transform.translation.x = butter_lowpass_filter(offset.transform.translation.x, cutoff, fs, order)
        offset.transform.translation.y = butter_lowpass_filter(offset.transform.translation.y, cutoff, fs, order)
        offset.transform.translation.z = butter_lowpass_filter(offset.transform.translation.z, cutoff, fs, order)
        #offset.transform.rotation.x = butter_lowpass_filter(offset.transform.rotation.x, cutoff, fs, order)
        #offset.transform.rotation.y = butter_lowpass_filter(offset.transform.rotation.y, cutoff, fs, order)
        #offset.transform.rotation.z = butter_lowpass_filter(offset.transform.rotation.z, cutoff, fs, order)
        #offset.transform.rotation.w = butter_lowpass_filter(offset.transform.rotation.w, cutoff, fs, order)
        """
        #print("after filter " + str(offset.transform.translation.x) + " " + str(offset.transform.translation.y) + " " + str(offset.transform.translation.z))
                      


        #===================================== 
        #offset_hist.append((offset.transform.translation.x,offset.transform.translation.y,offset.transform.translation.z))
        #offset_hist.append(offset.transform.translation.z)

        # pp.pprint(offset_hist)
        # print('list\n')
        #print(offset_hist)
        # if len(offset_hist)>1:
        #     offset_hist=offset_hist[-1:]  
        #     #print(offset_hist)
        #     average = np.mean(offset_hist, axis=0)
        #     #print(average)
        #     offset.transform.translation.x =average[0]
        #     offset.transform.translation.y =average[1]
        #     offset.transform.translation.z =average[2]
        #     #offset.transform.translation.y = average[1]  
        # #     print('hi!')
        # #     Fdiff=np.average(offset_hist)
        # #     print('list 2\n')
        # #     pp.pprint(Fdiff)
        # #     #sum(offset_hist)/float(len(offset_hist))
        #     del offset_hist[:]
        #     offset.transform.translation.z = 0
        # #     #=====================================
            
        #     tf_bc.sendTransform(offset)
        #     map2odom=offset
        #     localize.data=True
        #     pub_tf.publish(localize) 
        #     #=====================================
                
        #offset.transform.translation.z = 0 # Not considering any changes in z-axis

        filtered_offset = copy.deepcopy(offset)

            # ########Update the Kalman filter
        rot_ang = np.array([filtered_offset.transform.rotation.x, filtered_offset.transform.rotation.y, filtered_offset.transform.rotation.z, filtered_offset.transform.rotation.w], np.float32).reshape(-1, 1)
        translation = np.array([filtered_offset.transform.translation.x, filtered_offset.transform.translation.y], np.float32).reshape(-1, 1)

        #                             #    x,y,z       roll,pitch,yaw
        measurements = np.concatenate((translation, rot_ang))
        prediction = estimated_tf.predict()
        estimation = estimated_tf.correct(measurements.reshape(-1,1))
        
        offset.transform.translation.x =estimation[0]
        offset.transform.translation.y = estimation[1]    
        
        offset.transform.translation.x =estimation[0]
        offset.transform.translation.y = estimation[1]
        offset.transform.translation.z = 0
        offset.transform.rotation.x = estimation[2]
        offset.transform.rotation.y = estimation[3]
        offset.transform.rotation.z = estimation[4]
        offset.transform.rotation.w = estimation[5]
        n = np.linalg.norm([offset.transform.rotation.x, offset.transform.rotation.y, offset.transform.rotation.z, offset.transform.rotation.w])
        # #Normalize the quaternion
        offset.transform.rotation.x /= n
        offset.transform.rotation.y /= n
        offset.transform.rotation.z /= n
        offset.transform.rotation.w /= n 
        #'''
        #tf_bc.sendTransform(offset)
        map2odom=offset
        localize.data=True
        pub_tf.publish(localize) 

        # tf_bc.sendTransform(offset)
        # localize.data=True
        # pub_tf.publish(localize)
    #=============================================================================

    #=============================================================================

    

def LoadMapMarkers():    
    global json_markers
    #Read map info from json file
    mapFilePath = "/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/src/course_packages/dd2419_resources/worlds_json/awesome.world.json" #please fix
    mapString = ""

    with open(mapFilePath, "r") as file:
        for line in file:                           #for each row
            l = line.strip().replace(" ", "")       #remove all blankspace
            mapString += l

    mapInfo = ast.literal_eval(mapString)       #convert string representation of read file into dictionary through some kind of black magic

    temp_json_markers = mapInfo["markers"]
    
    for marker in temp_json_markers:
        t = Pose()
        [t.position.x, t.position.y, t.position.z] = marker["pose"]["position"]
        roll =  marker["pose"]["orientation"][0] 
        pitch = marker["pose"]["orientation"][1]
        yaw =   marker["pose"]["orientation"][2] 
        [t.orientation.x, t.orientation.y, t.orientation.z, t.orientation.w] = quaternion_from_euler(roll*math.pi/180, math.pi*pitch/180, math.pi*yaw/180)

        json_markers[marker["id"]] = t

def main():
    global map2odom
    LoadMapMarkers()

    rate = rospy.Rate(10)  # Hz
    while not rospy.is_shutdown():
        if markers and estimated_poses.keys()!='':
            
            transforms=[greta(estimated_poses[str(m)]) for m in estimated_poses.keys()]
            
            for i in range(len(transforms)):
                if transforms[i] !=0:
                    try:
                        tf_bc.sendTransform(transforms[i]) 
                        Map2localization(transforms[i])
                    except:
                        pass
        map2odom.header.stamp = rospy.Time.now()
        tf_bc.sendTransform(map2odom)
        #print(map2odom)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('localization',anonymous=True)
   
    sub_markers = rospy.Subscriber('/aruco/markers', MarkerArray, markers_callback)
    pub_tf= rospy.Publisher('/cf1/localize', Bool, queue_size=5)
    # Create buffer for tfs.
    buff = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buff)    
    tf_bc = tf2_ros.TransformBroadcaster()
    map2odom = TransformStamped()
    map2odom.header.stamp = rospy.Time.now()
    map2odom.header.frame_id = 'map'
    map2odom.child_frame_id = 'cf1/odom' 
    map2odom.transform.rotation.w=1
    tf_bc.sendTransform(map2odom)
    main()

