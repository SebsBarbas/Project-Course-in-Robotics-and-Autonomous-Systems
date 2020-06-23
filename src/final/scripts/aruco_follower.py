#!/usr/bin/env python
""" 
Group:5
Task= Milestone 2 - Aruco pose estimation.
"""

import rospy
import tf2_ros
import math
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped,TransformStamped,PoseWithCovarianceStamped
from crazyflie_driver.msg import Position
from aruco_msgs.msg import MarkerArray
from nav_msgs.msg import *
from sensor_msgs.msg import Imu

aruco_detected=False
aruco = None
drone_pos=PoseStamped()
goaltf=TransformStamped();
static=False

 # We detect an Aruco and this callback kicks in.
def arucoPose(aruco_array):
    #We get an array of markers 
    global aruco,goaltf,static
    aruco=[]
    transform=TransformStamped()
    base=PoseStamped()
    #For each marker
    for m in range (len(aruco_array.markers)):
        transform.header.frame_id='cf1/camera_link'
        transform.child_frame_id='aruco/detected'+str(aruco_array.markers[m].id)
        transform.transform.translation=aruco_array.markers[m].pose.pose.position
        transform.transform.rotation=aruco_array.markers[m].pose.pose.orientation
        aruco.append(transform)
        #rospy.loginfo('/aruco/detected'+str(aruco_array.markers[m].id))
        
        base.header.frame_id=transform.header.frame_id
        base.header.stamp=rospy.Time.now()
        base.pose.position=transform.transform.translation
        base.pose.orientation=transform.transform.rotation
        if tf_buff.can_transform(base.header.frame_id, 'cf1/base_link', base.header.stamp, rospy.Duration(0.5)): 
            base = tf_buff.transform(base, 'cf1/base_link')
            
        if((0.4<=base.pose.position.x<=0.7) and (-0.05<=base.pose.position.y<=0.05) and(-0.05<=base.pose.position.z<=0.05)):
            #We always ensure having a tf to goal pose.
            try:
                goaltf= tf_buff.lookup_transform(aruco[m].child_frame_id,'cf1/base_link', rospy.Time.now(), rospy.Duration(0.5))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass
            goaltf.child_frame_id='goal_pos'
            goaltf.header.frame_id=aruco[m].child_frame_id
            tf_bc.sendTransform(goaltf)
            static=True
    if(static):
        print(goaltf)
        goaltf.header.stamp=rospy.Time.now()
        tf_bc.sendTransform(goaltf)

def publishArucoTf(aruco):  
    global aruco_detected
    # Need to tell TF that the goal was just generated
    if aruco!=None:
        for m in range(len(aruco)):
            aruco[m].header.stamp=rospy.Time.now()
            tf_bc.sendTransform(aruco[m])
        aruco_detected=True
    aruco=None
    
    
def positionAruco():
    global tf_buff,aruco_detected

    trans=TransformStamped()
    try:
        # We want to keep the relative position.... We can calculate the error betwen these frames.
        trans = tf_buff.lookup_transform('cf1/odom','goal_pos', rospy.Time(0),rospy.Duration(0.5))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        pass

    # We should now have the translation and orientation of the aruco marker - the offset in x in odom frame... lets move now.
    cmd=Position()
    cmd.header.stamp = rospy.Time.now()
    cmd.header.frame_id = trans.header.frame_id
    cmd.x = trans.transform.translation.x
    cmd.y = trans.transform.translation.y
    cmd.z = trans.transform.translation.z
    roll,pitch,yaw = euler_from_quaternion((trans.transform.rotation.x,
                                                        trans.transform.rotation.y,
                                                        trans.transform.rotation.z,
                                                        trans.transform.rotation.w))
    cmd.yaw=math.degrees(yaw)    
    # We want to rotate opposite to the direction that the aruco rotated, else we would mirror the yaw. We should then be able to translate.
    pub_cmd.publish(cmd)

    
    
def pos_callback(msg):
    global drone_pos
    drone_pos=msg   

def main():
    rate = rospy.Rate(60)  # Hz
    while not rospy.is_shutdown():
        if aruco:
            publishArucoTf(aruco)
            positionAruco()
        rate.sleep()
    

if __name__ == '__main__':
    rospy.init_node('ArucoFollower')
    aruco_estimate=rospy.Subscriber('/aruco/markers',MarkerArray,arucoPose,queue_size = 1)
    dr_pos=rospy.Subscriber('/cf1/pose', PoseStamped, pos_callback)
    pub_cmd = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
    tf_bc=  tf2_ros.TransformBroadcaster()
    # Create the buffer for the transforms thta we will listen to:
    tf_buff=tf2_ros.Buffer()
    listener=tf2_ros.TransformListener(tf_buff)
    
    main()