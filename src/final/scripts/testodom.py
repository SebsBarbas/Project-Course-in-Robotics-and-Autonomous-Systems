#!/usr/bin/env python
# --- Milestone 2 ---
import rospy
import tf2_ros
import math
import tf2_geometry_msgs
import numpy as np
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from geometry_msgs.msg import PoseStamped,TransformStamped,PoseWithCovarianceStamped
from crazyflie_driver.msg import Position
from aruco_msgs.msg import MarkerArray
from nav_msgs.msg import *
from sensor_msgs.msg import Imu


def main():
    #global drone_pos
    setpoint=PoseStamped()
    fram=TransformStamped()
    setpoint.header.frame_id='map'
    setpoint.header.stamp=rospy.Time.now()
    print(drone_pos)

    setpoint.pose.position.x=drone_pos.pose.position.x#-0.25
    setpoint.pose.position.y=drone_pos.pose.position.y#-0.25
    setpoint.pose.position.z=drone_pos.pose.position.z#0.3
    
    setpoint.pose.orientation.x=drone_pos.pose.orientation.x#0
    setpoint.pose.orientation.y=drone_pos.pose.orientation.y#0
    setpoint.pose.orientation.z=drone_pos.pose.orientation.z
    setpoint.pose.orientation.w=drone_pos.pose.orientation.w
    cmd = Position()

    rate = rospy.Rate(20)  # Hz
    while not rospy.is_shutdown():
        print('Point in map\n')
        #print(setpoint)
        print('============================')
        if not buff.can_transform('map','cf1/odom', rospy.Time.now(), rospy.Duration(0.5)):
            rospy.logwarn_throttle(5.0 , 'No tranform from %s to map' % 'odom')
            return
        else:
            t= buff.transform(setpoint, 'cf1/odom')  
            
            fram.header.stamp = rospy.Time.now()
            fram.header.frame_id = 'cf1/odom'
            fram.child_frame_id =  'goal'
            fram.transform.translation.x = drone_pos.pose.position.x
            fram.transform.translation.y = drone_pos.pose.position.y
            fram.transform.translation.z = drone_pos.pose.position.z
            fram.transform.rotation.x = drone_pos.pose.orientation.x
            fram.transform.rotation.y = drone_pos.pose.orientation.y
            fram.transform.rotation.z = drone_pos.pose.orientation.z
            fram.transform.rotation.w = drone_pos.pose.orientation.w
            tf_bc.sendTransform(fram) 
                
            cmd.header.stamp = rospy.Time.now()
            cmd.header.frame_id = fram.header.frame_id
            cmd.x = drone_pos.pose.position.x
            cmd.y = drone_pos.pose.position.y
            cmd.z = 0.3

            _, _, yaw = euler_from_quaternion((drone_pos.pose.orientation.x,
                                                    drone_pos.pose.orientation.y,
                                                    drone_pos.pose.orientation.z,
                                                    drone_pos.pose.orientation.w))
            
            cmd.yaw = math.degrees(yaw)
            pub_cmd.publish(cmd)
        print('Point in odom\n')
        print(fram)
        print('============================')

        rate.sleep()
def pos_callback(msg):
    global drone_pos
    drone_pos=msg
if __name__ == '__main__':
    drone_pos=PoseStamped()
    rospy.init_node('testodom',anonymous=True)
    pub_cmd = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
    dr_pos=rospy.Subscriber('/cf1/pose', PoseStamped, pos_callback)
    buff = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buff)    
    tf_bc = tf2_ros.TransformBroadcaster()

    
    main()


