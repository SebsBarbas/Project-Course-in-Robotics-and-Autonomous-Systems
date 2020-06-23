#!/usr/bin/env python
""" 
Final object detection module with changes.
Group 5

"""
from __future__ import print_function
import timeit
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
from std_msgs.msg import Bool


import numpy as np

mtx=np.array([[225.93036228, 0.0, 323.39508531], [0.0, 223.29467374, 234.78851706], [0.0, 0.0, 1.0]])

dist=np.array([0.18928948, -0.18577923, -0.00287653, -0.00739736, 0.04164822])

def butter_lowpass(cutoff, fs, order=5):
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        return b, a


def butter_lowpass_filter(data, cutoff, fs, order, a, b):
        y = lfilter(b, a, data)
        return y

class ObjectDetection:

  def __init__(self):
    self.net=cv2.dnn.readNetFromTensorflow('/home/alsarmi/Desktop/tensorflow_training/all_signs/frozen_inference_graph.pb', '/home/alsarmi/Desktop/all_signs.pbtxt')
    self.image_pub = rospy.Publisher("/myresult", Image, queue_size=5)
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
 

  
  def callback(self,data):
    global mobilenet
    # Convert the image from OpenCV to ROS format
    if mobilenet:
      tic=timeit.default_timer()
      try:
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      except CvBridgeError as e:
        print(e)
      """
        Fancy code here.
      """

      # Get the filter coefficients so we can check its frequency response.
      global mtx, dist
      h,  w = cv_image.shape[:2]
      mtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

      blob = cv2.dnn.blobFromImage(cv_image, 1.5, size=(300, 300), swapRB=True)#, crop=True)
      self.net.setInput(blob)

      #Prediction of network
      detections = self.net.forward()

      cols = 300
      rows = 300

      for i in range(detections.shape[2]):
          confidence = detections[0, 0, i, 2] #Confidence of prediction 
          if confidence > 0.9: # Filter prediction 
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
                  
                  image_points =np.array([(xRightTop, yRightTop),
                                          (xRightTop-(xRightTop-xLeftBottom), yRightTop),
                                          (xLeftBottom+(xRightTop-xLeftBottom), yLeftBottom),
                                          (xLeftBottom, yLeftBottom)                                                           
                                          ],dtype=np.float32)
                                          
                  try:
                     _, rmat, tmat= cv2.solvePnP(self.model_points,image_points, mtx,dist,flags=6)[:3]#,iterationsCount = 500,reprojectionError = 2.0,confidence = 0.99)[:3]

                     #Update the Kalman filter
                     RotMat=cv2.Rodrigues(rmat)[0]
                     pose=np.hstack((RotMat,tmat))
                     pose=np.vstack((pose,[0,0,0,1])) 
                     _,invpose=cv2.invert(pose)
                     rot=invpose[0:3,0:3]
                     #print('Pose\n',invpose[0:3,3:4]) 
                     tmat=invpose[0:3,3:4]               
                     rmat=cv2.Rodrigues(rot)[0]
                        
                     if np.linalg.norm(tmat) < 20:
                         transform=TransformStamped()
                         transform.header.frame_id='cf1/camera_link'
                         transform.child_frame_id='sign/'+ self.classNames[class_id]
                         transform.header.stamp=rospy.Time.now()

                         #transform.transform.translation.x = tmat[2]#tmat[2]/950#+0.2
                         #transform.transform.translation.y = tmat[1]
                         #transform.transform.translation.z = tmat[0]
                         transform.transform.translation.x=abs(butter_lowpass_filter(tmat[0], cutoff, fs, order, a, b)*100*3.3-1.13) #+0.2#tmat[2]/950#+0.2
                         transform.transform.translation.y=(butter_lowpass_filter(tmat[1], cutoff, fs, order, a, b)*100*(0.88)+0.44)/2 #-0.1#tmat[1]/950#-0.1
                         transform.transform.translation.z=butter_lowpass_filter(tmat[2], cutoff, fs, order, a, b)*100*(-0.25)-0.0298#*1.5
                          
                         #a=quaternion_from_euler(rmat[0]-math.pi,rmat[1],rmat[2]+math.pi)
                         #a=quaternion_from_euler(rmat[0]*math.pi/180,rmat[1]*math.pi/180+math.pi,rmat[2]*math.pi/180-math.pi/2)
                  #       rot=quaternion_from_euler(rmat[0]*math.pi/180,rmat[1]*math.pi/180,rmat[2]*math.pi/180)
                                           #       transform.transform.rotation.x=rot[0]
                  #       transform.transform.rotation.y=rot[1]
                  #       transform.transform.rotation.z=rot[2]
                  #       transform.transform.rotation.w=rot[3]
                         transform.transform.rotation.w=1
                  #       #print(transform)
                         if transform.transform.translation.x < 0.94:
                  #           #rate.sleep()
                             tf_bc.sendTransform(transform) 
                  #           #print(transform)
                  except :
                     print("FAILED!!!!!!!!!!!!")
                  #   continue  
      """
      Fancy code finishes here.
      """
      # Publish the image
      try:
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image,"bgr8"))#np.asarray(thresh_img),"mono8"))
      except CvBridgeError as e:
        print(e)
      toc=timeit.default_timer()
      dif=toc-tic
      print('Elapsed time: \n',dif)
      mobilenet=False
      
def main(args):
  

  ic = ObjectDetection()
  
  print("running...")
  while not rospy.is_shutdown():
    try:
      rospy.spin()
      #rate.sleep()
    except KeyboardInterrupt:

      print("Shutting down")

  #cv2.destroyAllWindows()

def detect_cb(msg):
    global mobilenet
    mobilenet=msg.data
    print('Received')
    
if __name__ == '__main__':
    
    rospy.init_node('object_detection_mobilenet', anonymous=True)
    rate = rospy.Rate(5)
    tf_bc=  tf2_ros.TransformBroadcaster()
    # Create the buffer for the transforms thta we will listen to:
    tf_buff=tf2_ros.Buffer()
    listener=tf2_ros.TransformListener(tf_buff)
    order = 6
    fs = 30.0       # sample rate, Hz
    cutoff = 5  # desired cutoff frequency of the filter, Hz
    mob=rospy.Publisher('/detect',Bool,queue_size=5)

    b, a = butter_lowpass(cutoff, fs, order)
    detect=rospy.Subscriber('/detect',Bool,detect_cb)
    mobilenet=False
    main(sys.argv)