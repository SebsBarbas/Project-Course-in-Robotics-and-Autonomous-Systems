#!/usr/bin/env python

# Import relevant messages
from geometry_msgs.msg import PoseStamped
from crazyflie_driver.msg import Position
# Import Transforms and Frames 
from tf.transformations import euler_from_quaternion
import tf2_geometry_msgs
import tf2_ros
# Import relevant tools
import rospy
import math

# Create goal
goal = None
rospy.init_node('move')
# Current position
pos_ = PoseStamped()
drone_pos=PoseStamped()

def pub(goal):
    pub_ = goal
    pub_.header.stamp = rospy.Time.now()

    if not buff.can_transform(pub_.header.frame_id, 'cf1/odom', pub_.header.stamp, rospy.Duration(0.5)):
        rospy.logwarn_throttle(5.0 , 'No tranform from %s to cf1/odom' % pub_.header.frame_id)
        return
    
    odo = buff.transform(pub_ , 'cf1/odom')
    cmd = Position()
    cmd.header.stamp = rospy.Time.now()
    cmd.header.frame_id = odo.header.frame_id
    cmd.x = odo.pose.position.x
    cmd.y = odo.pose.position.y
    cmd.z = odo.pose.position.z

    _, _, yaw = euler_from_quaternion((odo.pose.orientation.x,
                                            odo.pose.orientation.y,
                                            odo.pose.orientation.z,
                                            odo.pose.orientation.w))
    cmd.yaw = math.degrees(yaw)
    pub_cmd.publish(cmd)

def callback_keyboard(msg):
    global pos_

    pos_.pose.position.x = msg.pose.position.x 
    pos_.pose.position.y =  msg.pose.position.y 
    pos_.pose.orientation.z = msg.pose.orientation.z 
    pos_.pose.orientation.x = msg.pose.orientation.x 
    pos_.pose.orientation.y = msg.pose.orientation.y 
    pos_.pose.orientation.w = msg.pose.orientation.w 
    print(pos_)


def callback_pose(msg):
    global pos_
    pos_ = msg
    print(pos_)

def callback_check_routine(msg):
    global pos_
    pos_ = msg
    print(pos_)

def callback_PathPlanning(msg):
    global pos_
    pos_ = msg
    print(pos_)
    
def pos_callback(msg):
    global drone_pos
    drone_pos=msg

# Create Publisher
pub_cmd = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
# Create Subscriber to keyboard
subscriber_key = rospy.Subscriber('/cf1/keyboard', PoseStamped, callback_keyboard)
subscriber_pose = rospy.Subscriber('/cf1/keyboard_pose', PoseStamped, callback_pose)
dr_pos=rospy.Subscriber('/cf1/pose', PoseStamped, pos_callback)
# Create Subscriber to PathPlanning
subscriber_pp = rospy.Subscriber('/cf1/PathPlanning', PoseStamped, callback_PathPlanning)
# Create Subscriber to Checkpoint routine
subscriber_cr = rospy.Subscriber('/cf1/check_routine', PoseStamped, callback_check_routine)
# Create buffer
buff = tf2_ros.Buffer()
buff2 = tf2_ros.TransformListener(buff)

def main():
    global pos_
    rate = rospy.Rate(10) # 10 Hz
    rate.sleep() 
    pos_ = PoseStamped()
    pos_.header.frame_id = 'map'
    # Initialize Position
    pos_.pose.position.x = drone_pos.pose.position.x
    pos_.pose.position.y = drone_pos.pose.position.x
    pos_.pose.position.z = 0.6
    # Initialize Rotation
    pos_.pose.orientation=drone_pos.pose.orientation

    while not rospy.is_shutdown():
        pub(pos_)
        rate.sleep() # Remember to sleep


if __name__ == '__main__':
    main()