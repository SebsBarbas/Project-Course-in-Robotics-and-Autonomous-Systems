#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

if __name__ == '__main__':
    rospy.init_node('crazyflie_demo_const_thrust', anonymous=True)
    p = rospy.Publisher('cf1/cmd_vel', Twist,queue_size=2.0)
    twist = Twist()
    r = rospy.Rate(50)
    #for i in range(0, 100):
    #    p.publish(twist)
    #    r.sleep()

    twist.linear.z = 12000
    while not rospy.is_shutdown():
        p.publish(twist)
        r.sleep()
