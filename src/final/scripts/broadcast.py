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


# Global variables--- Avoid using them...

aruco = None
drone_pos=PoseStamped()
goaltf=TransformStamped()
cmd_pos=Position()
static=False
init=False



#=======================================================================================
#--------------------------------------- Regular publishing of received pose
#=======================================================================================

def pub(goal):
    global cmd_pos
    pub_ = goal
    pub_.header.stamp = rospy.Time.now()
    print("Before transformation\n")
    print(pub_)
    print("=============================\n")

    if(pub_.header.frame_id!=''):
       # pub_.header.frame_id='map'
        print('Before sending: ')
        if(pub_.header.frame_id!='cf1/odom'):
            if not buff.can_transform(pub_.header.frame_id, 'cf1/odom', pub_.header.stamp, rospy.Duration(0.5)):
                rospy.logwarn_throttle(5.0 , 'No tranform from %s to cf1/odom' % pub_.header.frame_id)
                return
            odo = buff.transform(pub_ , 'cf1/odom')
        else:
            odo=pub_
    # print(odo)
        cmd = Position()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = odo.header.frame_id
        cmd.x = odo.pose.position.x
        cmd.y = odo.pose.position.y
        cmd.z = 0.35

        _, _, yaw = euler_from_quaternion((odo.pose.orientation.x,
                                                odo.pose.orientation.y,
                                                odo.pose.orientation.z,
                                                odo.pose.orientation.w))
        
        cmd.yaw = math.degrees(yaw)
        print('Before sending: ')
        print(cmd)
        pub_cmd.publish(cmd)

def pub_map_pos(goal):
    
    odom_pos= goal
    odom_pos.header.stamp = rospy.Time.now()

    if(odom_pos.header.frame_id!=''):
     
        if not buff.can_transform(odom_pos.header.frame_id, 'map', odom_pos.header.stamp, rospy.Duration(0.5)):
            rospy.logwarn_throttle(5.0 , 'No tranform from %s to map' % odom_pos.header.frame_id)
            return
        map_pos = buff.transform(odom_pos , 'map')
    
    # print(map_pos)
        cmd = Position()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = map_pos.header.frame_id
        cmd.x = map_pos.pose.position.x
        cmd.y = map_pos.pose.position.y
        cmd.z = map_pos.pose.position.z

        _, _, yaw = euler_from_quaternion((map_pos.pose.orientation.x,
                                                map_pos.pose.orientation.y,
                                                map_pos.pose.orientation.z,
                                                map_pos.pose.orientation.w))
        
        cmd.yaw = math.degrees(yaw)
        drone_pos_map.publish(cmd)
        
def broadcast_pose(msg):
    global pos_, init
    print("Del topico cf1/hold_pos \n")
    pos_ = msg
    
    init=True

def broadcast_pose_cmd(msg):
    global pos_
    pos_=PoseStamped()
    pos_.header.frame_id=msg.header.frame_id
    pos_.header.stamp=rospy.Time.now()
    pos_.pose.position.x=msg.x
    pos_.pose.position.y=msg.y
    pos_.pose.position.z=msg.z
    q=quaternion_from_euler(0.0,0.0,msg.yaw)
    pos_.pose.orientation.x=q[0]
    pos_.pose.orientation.y=q[1]
    pos_.pose.orientation.z=q[2]
    pos_.pose.orientation.w=q[3]
    print('De cf1/hold_pos_cmd\n')
    print(pos_)
    """
    print('QUATERNION FROM HELL')
    print(pos_)
    print('QUATERNION FROM HELL2')
    """

    """
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
    """
    #pub_cmd.publish(msg)
    
def pos_callback(msg):
    global drone_pos
    drone_pos=msg
    
def transform_broadcast(tfmsg):
    tf_bc.sendTransform(tfmsg)
    
def main():
    global pos_, drone_pos, init
    rate = rospy.Rate(10) # 10 Hz
    rate.sleep()
    #pos_ = PoseStamped()
    #pos_.header.frame_id = 'map'
    
    # Initialize Position
    #pos_.pose.position.x = drone_pos.pose.position.x
    #pos_.pose.position.y = drone_pos.pose.position.y
    #pos_.pose.position.x = -0.1
    #pos_.pose.position.y = -0.1
    #pos_.pose.position.z = 0.3 
    # Initialize Rotation
    #pos_.pose.orientation=drone_pos.pose.orientation
    

    while not rospy.is_shutdown():
        #print(pos_)
        print(pos_)
        if pos_ != None:
            pub(pos_)
            pub_map_pos(drone_pos)
           # print('Print in main\n',pos_)
        rate.sleep() # Remember to sleep
        
#=======================================================================================
#--------------------------------------- Main
#=======================================================================================

if __name__ == '__main__':
    # Create goal
    goal = None
    rospy.init_node('broadcast')
    # Current position
    pos_ = None
    drone_pos=PoseStamped()
    # Create Publisher
    pub_cmd = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
    # Create Publisher
    # pub_tree = rospy.Publisher('arucoFollow', Position, queue_size=2)
    # Create Subscriber to hold_pose topic.
    subscriber_key = rospy.Subscriber('/cf1/hold_pos', PoseStamped, broadcast_pose) #Only sending the received position which has to be in position message

    subscriber_key_cmd = rospy.Subscriber('/cf1/hold_pos_cmd', Position, broadcast_pose_cmd) #Transforms from PoseStamped to Position
    drone_pos_map  = rospy.Publisher('/cf1/pose_map', Position, queue_size=2)
    # Create Subscriber to pose of the drone.
    dr_pos=rospy.Subscriber('/cf1/pose', PoseStamped, pos_callback)
    # Create Subscriber to tf topic listener.
    tftopic=rospy.Subscriber('tf_broadcast',TransformStamped,transform_broadcast)
    # Create Subscriber to aruco pose topic
    #aruco_estimate=rospy.Subscriber('/aruco/markers',MarkerArray,arucoPose,queue_size = 1)

    # Create buffer for tfs.
    buff = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buff)    
    tf_bc = tf2_ros.TransformBroadcaster()

    main()