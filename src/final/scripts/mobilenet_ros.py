#!/usr/bin/env python
""" 
Final object detection module with changes.
Group 5

"""
from __future__ import print_function
import imutils
import rospy 
import sys 
import tf
import tf2_ros
import math
from tf.transformations import *
import roslib
import cv2 
import numpy as np
from matplotlib import pyplot as plt
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped,TransformStamped,PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from scipy.signal import butter, lfilter, freqz

import pickle

import numpy as np

mtx=np.array([[225.93036228, 0.0, 323.39508531], [0.0, 223.29467374, 234.78851706], [0.0, 0.0, 1.0]])

dist=np.array([0.18928948, -0.18577923, -0.00287653, -0.00739736, 0.04164822])

def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a


def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y


class ObjectDetection:


  def unpickle_keypoints(self,array):
    keypoints = []
    descriptors = []
    for point in array:
        temp_feature = cv2.KeyPoint(x=point[0][0],y=point[0][1],_size=point[1], _angle=point[2], _response=point[3], _octave=point[4], _class_id=point[5])
        temp_descriptor = point[6]
        keypoints.append(temp_feature)
        descriptors.append(temp_descriptor)
    return keypoints, np.array(descriptors)

  def __init__(self):
    self.net=cv2.dnn.readNetFromTensorflow('/home/alsarmi/Desktop/tensorflow_training/all_signs/frozen_inference_graph.pb', '/home/alsarmi/Desktop/all_signs.pbtxt')
    self.folder='/home/alsarmi/Desktop/SebastianORB/object_detection_demo/PnPRansac/objects/'
    self.image_pub = rospy.Publisher("/myresult", Image, queue_size=2)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/cf1/camera/image_raw", Image, self.callback)
    self.scale=0.2
    #Labels of network.
    print(cv2.__version__)
    self.model_points =np.array([(-1,1,0),
                                (1,1,0),
                                (1,-1,0),
                                (-1,-1,0)],dtype=np.float32)
    self.classNames = { 1: 'airport', 2: 'dangerous_curve_left', 3: 'dangerous_curve_right', 4: 'follow_left',
    5: 'follow_right', 6: 'junction', 7: 'no_bicycle', 8: 'no_heavy_truck', 9: 'no_parking',
    10: 'no_stopping_and_parking', 11: 'residential', 12: 'road_narrows_from_left', 13: 'road_narrows_from_right',
    14: 'roundabout_warning', 15: 'stop' }
    self.images = ['airport.png', 'dangerous_curve_left.png', 'dangerous_curve_right.png', 'follow_left.png',
              'follow_right.png', 'junction.png',
              'no_bicycle.png', 'no_heavy_truck.png', 'no_parking.png', 'no_stopping_and_parking.png',
              'residential.png', 'road_narrows_from_left.png',
              'road_narrows_from_right.png', 'roundabout_warning.png', 'stop.png']
    with open('/home/alsarmi/Desktop/SebastianORB/object_detection_demo/PnPRansac/keypoints_database.p', 'rb') as f:
      self.keypoints_database  = pickle.load(f) 

    self.dict_ekf ={}
    # FLANN parameters
    FLANN_INDEX_LSH = 6
    index_params= dict(algorithm = FLANN_INDEX_LSH,
                      table_number = 6, # 12
                      key_size = 12,     # 20
                      multi_probe_level = 1) #2
    search_params = dict(checks=100)   # or pass empty dictionary
    self.flann = cv2.FlannBasedMatcher(index_params,search_params)
    self.orb = cv2.ORB_create()

  
  def callback(self,data):
    # Convert the image from OpenCV to ROS format
    #self.fps = FPS().start()
    
    # Filter requirements.
    order = 6
    fs = 30.0       # sample rate, Hz
    cutoff = 3.3  # desired cutoff frequency of the filter, Hz
    # Get the filter coefficients so we can check its frequency response.
    b, a = butter_lowpass(cutoff, fs, order)
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    """
      Fancy code here.
    """
    global mtx, dist
    h,  w = cv_image.shape[:2]
    mtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

    blob = cv2.dnn.blobFromImage(cv_image, 1.5, size=(300, 300), swapRB=True, crop=True)
    self.net.setInput(blob)

    #Prediction of network
    detections = self.net.forward()

    cols = 300
    rows = 300

    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2] #Confidence of prediction 
        if confidence > 0.8: # Filter prediction 
            class_id = int(detections[0, 0, i, 1]) # Class label
            # Object location 
            xLeftBottom = int(detections[0, 0, i, 3] * cols) 
            yLeftBottom = int(detections[0, 0, i, 4] * rows)
            xRightTop   = int(detections[0, 0, i, 5] * cols)
            yRightTop   = int(detections[0, 0, i, 6] * rows)
            # Factor for scale to original size of frame
            heightFactor = cv_image.shape[0]/300.0  
            widthFactor = cv_image.shape[1]/300.0 
            # Scale object detection to frame
            xLeftBottom = int(widthFactor * xLeftBottom) 
            yLeftBottom = int(heightFactor * yLeftBottom)
            xRightTop   = int(widthFactor * xRightTop)
            yRightTop   = int(heightFactor * yRightTop)
                        
            roi = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            roi = roi[yLeftBottom:yRightTop, xLeftBottom:xRightTop]
 
            # Draw location of object  
            cv2.rectangle(cv_image, (xLeftBottom, yLeftBottom), (xRightTop, yRightTop),
                          (0, 255, 0))
            if class_id in self.classNames:
                label = self.classNames[class_id] + ": " + str(confidence)
                labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                yLeftBottom = max(yLeftBottom, labelSize[1])
                cv2.rectangle(cv_image, (xLeftBottom,  yLeftBottom - labelSize[1]),
                                    (xLeftBottom + labelSize[0], yLeftBottom + baseLine),
                                    (0, 255, 0), cv2.FILLED)
                cv2.putText(cv_image, label, (xLeftBottom, yLeftBottom),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))
                # Use the box of the detected object for the translation estimation.
                ''''
                image_points =np.array([(xRightTop, yRightTop),
                                        (xRightTop-(xRightTop-xLeftBottom), yRightTop),
                                        (xLeftBottom+(xRightTop-xLeftBottom), yLeftBottom),
                                        (xLeftBottom, yLeftBottom)                                                           
                                        ],dtype=np.float32)
                                        '''

                #_,rmat,tmat= cv2.solvePnPRansac(self.model_points,image_points, mtx,dist,flags=6,iterationsCount = 500,reprojectionError = 2.0,confidence = 0.99)[:3]
              
                #success,rmat,tmat= cv2.solvePnPRansac(self.model_points,image_points, mtx,dist,flags=6)[:3]#,iterationsCount = 500,reprojectionError = 2.0,confidence = 0.95)[:3]
              
              
              #Then check ORB matching for the orientation.
                kp_model, desc_model = self.unpickle_keypoints(self.keypoints_database[class_id-1])
               # print(desc_model)
                if str(class_id) not in self.dict_ekf.keys():
                      
                      #Kalman Filter for the pose
                      self.dict_ekf[str(class_id)] = cv2.KalmanFilter(6, 6)
                      trans_mat = np.identity(6, np.float32)
                      self.dict_ekf[str(class_id)].transitionMatrix = trans_mat                    
                      self.dict_ekf[str(class_id)].measurementMatrix = trans_mat 
                      self.dict_ekf[str(class_id)].processNoiseCov = cv2.setIdentity(self.dict_ekf[str(class_id)].processNoiseCov, 1e-1) #4     
                      self.dict_ekf[str(class_id)].measurementNoiseCov = cv2.setIdentity(self.dict_ekf[str(class_id)].measurementNoiseCov,1e-1)#2   
                      self.dict_ekf[str(class_id)].errorCovPost = cv2.setIdentity(self.dict_ekf[str(class_id)].errorCovPost)#, 1) 
  
                kp_image, desc_image = self.orb.detectAndCompute(roi, mask = None)

                try:

                  matches=self.flann.knnMatch(desc_model,desc_image,k=2)
                  matchesMask = [[0,0] for i in range(len(matches))]
                  #-- Filter matches using the Lowe's ratio test
                  good_matches = []
                  for i,(m,n) in enumerate(matches):
                      if m.distance < 0.7*n.distance:
                          matchesMask[i]=[1,0]
                          good_matches.append(m)
                  draw_params = dict(matchColor = (0,0,255), singlePointColor = (255,0,0),
                                  matchesMask = matchesMask,flags = cv2.DrawMatchesFlags_DEFAULT)
 
                  matches = good_matches #sorted(good_matches, key = lambda x: x.distance)
                  list_kpimage=[]
                  list_kpmodel=[]                      
                  for mat in matches:
                      img1_idx = mat.queryIdx
                      img2_idx = mat.trainIdx

                      (x1, y1) = kp_model[img1_idx].pt
                      (x2, y2) = kp_image[img2_idx].pt

                  # Append to each list

                      list_kpmodel.append((x1, y1 , 0.0))
                      list_kpimage.append((x2, y2))
                except:
                  continue
                
                image_points2 =np.array(list_kpimage, dtype = np.float32) 
                model_points2 =np.array(list_kpmodel, dtype = np.float32)
              
                try:
                  if not (image_points2.size == 0 and model_points2.size == 0 ):
                    
                    _, rmat, tmat= cv2.solvePnPRansac(model_points2, image_points2, mtx, dist,flags = 6,iterationsCount = 500,reprojectionError = 1.0,confidence = 0.99)[:3]
                    if np.linalg.norm(tmat) < 1000:
                      measurements = np.array(tmat,np.float32)#.reshape(1,-1)
                      prot=np.array(rmat,np.float32)#.reshape(1,-1)
                      measurements = np.concatenate((measurements, prot))
                      prediction = self.dict_ekf[str(class_id)].predict()
                      estimated = self.dict_ekf[str(class_id)].correct(measurements.reshape(-1,1))
                      rmat2=np.array(estimated[3:6])
                    
                    #Update the Kalman filter
                    RotMat=cv2.Rodrigues(rmat2)[0]
                    pose=np.hstack((RotMat,tmat))
                    pose=np.vstack((pose,[0,0,0,1])) 
                    _,invpose=cv2.invert(pose)
                    rot=invpose[0:3,0:3]
                    #print('Pose\n',invpose[0:3,3:4]) 
                    tmat=invpose[0:3,3:4]               
                    rmat=cv2.Rodrigues(rot)[0]
                    
                    if np.linalg.norm(tmat) < 10000:
                      transform=TransformStamped()
                      transform.header.frame_id='cf1/camera_link'
                      transform.child_frame_id='sign/'+ self.classNames[class_id]
                      transform.header.stamp=rospy.Time.now()

                      transform.transform.translation.x=round(butter_lowpass_filter(tmat[2], cutoff, fs, order),3) +0.2#tmat[2]/950#+0.2
                      transform.transform.translation.y=round(butter_lowpass_filter(tmat[1], cutoff, fs, order),3) -0.1#tmat[1]/950#-0.1
                      transform.transform.translation.z=round(butter_lowpass_filter(tmat[0], cutoff, fs, order),3)*1.5
                      
                      #a=quaternion_from_euler(rmat[0]-math.pi,rmat[1],rmat[2]+math.pi)
                      a=quaternion_from_euler(rmat[0]*math.pi/180,rmat[1]*math.pi/180+math.pi,rmat[2]*math.pi/180-math.pi/2)
                      transform.transform.rotation.x=a[0]
                      transform.transform.rotation.y=a[1]
                      transform.transform.rotation.z=a[2]
                      transform.transform.rotation.w=a[3]
                      #print(transform)
                      if transform.transform.translation.z>0.15:
                        tf_bc.sendTransform(transform)  
                except:

                  pass     
    """
    Fancy code finishes here.
    """
    # Publish the image
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image,"bgr8"))#np.asarray(thresh_img),"mono8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  

  ic = ObjectDetection()

  print("running...")
  try:
    rospy.spin()
  except KeyboardInterrupt:

    print("Shutting down")

  cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('object_detection_mobilenet', anonymous=True)
    tf_bc=  tf2_ros.TransformBroadcaster()
    # Create the buffer for the transforms thta we will listen to:
    tf_buff=tf2_ros.Buffer()
    listener=tf2_ros.TransformListener(tf_buff)
    main(sys.argv)