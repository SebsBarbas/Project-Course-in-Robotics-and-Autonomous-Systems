#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Bool
# Import relevant messages
from geometry_msgs.msg import PoseStamped
from crazyflie_driver.msg import Position
# Import Transforms and Frames 
from tf.transformations import euler_from_quaternion
import tf2_geometry_msgs
import tf2_ros
# Import relevant tools




def main():
    rate=rospy.Rate(0.5)
    while not rospy.is_shutdown():
        #inp=raw_input('Turn on and off mobilenet')
        rate.sleep()
        mobile.data=True#not mobile.data
        mobilenet.publish(mobile.data)
        #print(mobile.data)
if __name__ == '__main__':
    rospy.init_node('Control_mobilenet')
    mobilenet=rospy.Publisher('/detect',Bool,queue_size=5)
    mobile=Bool()

    main()