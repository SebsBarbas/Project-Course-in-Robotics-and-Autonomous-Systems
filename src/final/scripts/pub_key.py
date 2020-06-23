#!/usr/bin/env python

# --- Milestone 1, Publishing keyboard commands ---

# Import relevant messages
from geometry_msgs.msg import PoseStamped, Quaternion
# Import Transforms and Frames 
from tf.transformations import *
import tf2_geometry_msgs
import tf2_ros
# Import relevant tools
import rospy
import math
import numpy as np
from numpy import linalg as LA

rospy.init_node('pub_key')
# Current position
pos_ = PoseStamped()
yaw = 180
q_orig = quaternion_from_euler(0, 0, 0)
q_orig[3]=-q_orig[3]
experiment=None
drone_pos=PoseStamped()
def make_goal():
    global pos_, yaw,q_orig,experiment,drone_pos
    pos_.header.stamp = rospy.Time.now()
    print(' ')
    print('Press one of the available commands:')
    print(' ')
    print('   w ')
    print('a    d         (Or "r" for rotation)     (Or "E" to run experiment)')
    print('   s')
    print('q to exit')
    
    
    pos_.pose.orientation= drone_pos.pose.orientation
    print(pos_)
    
    #q_orig=np.array([drone_pos.pose.orientation.x, drone_pos.pose.orientation.y, drone_pos.pose.orientation.z, drone_pos.pose.orientation.w])


    fly = raw_input()
    if fly == 'w':
        pos_.pose.position.x =drone_pos.pose.position.x-0.1
        pos_.pose.position.y = drone_pos.pose.position.y
        pos_.pose.orientation= drone_pos.pose.orientation        
        print('Moving forward 0.5 steps.')
    if fly == 's':
        pos_.pose.position.x = drone_pos.pose.position.x+0.1
        pos_.pose.position.y = drone_pos.pose.position.y
        pos_.pose.orientation= drone_pos.pose.orientation
        print('Moving backwards 0.5 steps.')
    if fly == 'a':
        pos_.pose.position.y =drone_pos.pose.position.y -0.1
        pos_.pose.position.x = drone_pos.pose.position.x
        pos_.pose.orientation= drone_pos.pose.orientation
        print('Moving to the left 0.5 steps.')
    if fly == 'd':
        pos_.pose.position.y = drone_pos.pose.position.y+0.1
        pos_.pose.position.x = drone_pos.pose.position.x
        pos_.pose.orientation= drone_pos.pose.orientation
        print('Moving to the right 0.5 steps.')
    if fly=='u':
        pos_.pose.position.y = drone_pos.pose.position.y
        pos_.pose.position.x = drone_pos.pose.position.x
        pos_.pose.position.z = drone_pos.pose.position.z+0.05
        pos_.pose.orientation= drone_pos.pose.orientation
    if fly=='g':
        if(drone_pos.pose.position.z>0.2):
            pos_.pose.position.y = drone_pos.pose.position.y
            pos_.pose.position.x = drone_pos.pose.position.x
            pos_.pose.position.z = drone_pos.pose.position.z-0.05
            pos_.pose.orientation= drone_pos.pose.orientation
    if fly=='q':
        pos_.pose.position.z = 0.2
        
    if fly == 'r':
        yaw+=45
        q_rot = quaternion_from_euler(0.0 ,0.0 , math.radians(yaw))       
        pos_.pose.position.x = drone_pos.pose.position.x
        pos_.pose.position.y = drone_pos.pose.position.y
        q_new=quaternion_multiply(q_rot,q_orig)
        pos_.pose.orientation.x,pos_.pose.orientation.y,pos_.pose.orientation.z,pos_.pose.orientation.w = (q_new[0],q_new[1],q_new[2],q_new[3])

        print('rotating %i degrees'%yaw)
        print(pos_)
        
    if fly == 'E':
        experimen()
        
    if not experiment:
        print(pos_)
        pub.publish(pos_)
    experiment=False
    
def experimen():
    global pos_,drone_pos
    d=100
    rate=rospy.Rate(10000)
    pos_.pose.position.x=0
    pos_.pose.position.y=0
    
    pos_.header.stamp = rospy.Time.now()
    pos_.pose.orientation= drone_pos.pose.orientation
    q_orig = quaternion_from_euler(0, 0, 0)
    q_orig[3]=-q_orig[3]

    #q_orig=np.array([drone_pos.pose.orientation.x, drone_pos.pose.orientation.y, drone_pos.pose.orientation.z, drone_pos.pose.orientation.w])

    
    m= float(raw_input('How many meters do you want to move? \n'))

    spin=int(raw_input('How many spins do you want the drone to perform? \n'))
    print('Hello')
    spin=spin*4
    pos_.pose.position.x = m #+drone_pos.pose.position.x
    pos_.pose.position.y = 0#drone_pos.pose.position.y
    
    pub.publish(pos_)
    temp_pose=PoseStamped()
    temp_pose.pose.position.x=m
    
    while (d>0.06):
        d=math.sqrt((temp_pose.pose.position.x-drone_pos.pose.position.x)**2)
        print(d)
  
    d=100   
    rospy.loginfo('Reached position , with tolerance: %.4f'%d)
    yaw=180
    while spin>=0:
        spin-=1
        yaw+=90
        q_rot = quaternion_from_euler(0.0 ,0.0 , math.radians(yaw))
        pos_.pose.position.x = drone_pos.pose.position.x
        pos_.pose.position.y = drone_pos.pose.position.y
        q_new=quaternion_multiply(q_rot,q_orig)
        pos_.pose.orientation.x,pos_.pose.orientation.y,pos_.pose.orientation.z,pos_.pose.orientation.w = (q_new[0],q_new[1],q_new[2],q_new[3])
        pub.publish(pos_)
        
        
        print('SPIN')
        rospy.sleep(2.)
        '''         while((math.degrees(rpy2[2])-math.degrees(rpy[2]))>1):
            rpy=euler_from_quaternion([drone_pos.pose.orientation.x,drone_pos.pose.orientation.y,drone_pos.pose.orientation.z,drone_pos.pose.orientation.w])
           # rpy=euler_from_quaternion([drone_pos.pose.orientation.x,drone_pos.pose.orientation.y,drone_pos.pose.orientation.z,drone_pos.pose.orientation.w])
            rpyy=euler_from_quaternion([pos_.pose.orientation.x,pos_.pose.orientation.y,pos_.pose.orientation.z,pos_.pose.orientation.w])
            print('Yaw \n ',yaw)
            print('Pose \n',pos_)
            print('Yaw \n',rpyy[2])
            pub.publish(pos_)
            
            #print('RPY ',euler_from_quaternion(drone_pos.pose.orientation))
            rate.sleep() '''
    
    pos_.pose.position.x = 0
    pos_.pose.position.y = 0
    
    pub.publish(pos_)
    temp_pose=PoseStamped()
    temp_pose.pose.position.x=0
    
    while (d>0.04):
        d=math.sqrt((temp_pose.pose.position.x-drone_pos.pose.position.x)**2)
       
        
    rospy.loginfo('Reached position , with tolerance: %.4f'%d)
    print('Hello!')
    
    rate.sleep()
    
def pos_callback(msg):
    global drone_pos
    drone_pos=msg
    
# Create Publisher
pub = rospy.Publisher('/cf1/hold_pos', PoseStamped, queue_size=2)
dr_pos=rospy.Subscriber('/cf1/pose', PoseStamped, pos_callback)

def main():
    global pos_
    rate = rospy.Rate(10) # 10 Hz
    pos_ = PoseStamped()
    pos_.header.frame_id = 'map'
    # Initialize Position
    pos_.pose.position.x = 0
    pos_.pose.position.y = 0
    pos_.pose.position.z = 0.3
    # Initialize Rotation
    pos_.pose.orientation.x = 0
    pos_.pose.orientation.y = 0
    pos_.pose.orientation.z = 0
    pos_.pose.orientation.w = 0


    while not rospy.is_shutdown():
        make_goal()
        rate.sleep() # Remember to sleep


if __name__ == '__main__':
    main()