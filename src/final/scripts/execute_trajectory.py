#!/usr/bin/env python

import argparse
import rospy
import tf_conversions
from crazyflie_driver.msg import FullState
import geometry_msgs
import uav_trajectory


if __name__ == '__main__':
    # parser = argparse.ArgumentParser()
    # parser.add_argument("trajectory", type=str, help="CSV file containing trajectory")
    # args = parser.parse_args()

    rospy.init_node('full_state',anonymous=True)

    traj = uav_trajectory.Trajectory()
    traj.loadcsv('/home/alsarmi/Dropbox/KTH_Classes/Project_Course_Drone/dd2419_ws/src/milestone2/scripts/figure8withTakeoffAndLanding.csv')

    rate = rospy.Rate(100)

    msg = FullState()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"

    pub = rospy.Publisher("/cf1/cmd_full_state", FullState, queue_size=1)
    start_time = rospy.Time.now()

    while not rospy.is_shutdown():
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        t = (msg.header.stamp - start_time).to_sec()
        
        if t > traj.duration:
            break
            # t = traj.duration

        e = traj.eval(t)

        msg.pose.position.x    = e.pos[0]
        msg.pose.position.y    = e.pos[1]
        msg.pose.position.z    = e.pos[2]
        msg.twist.linear.x     = e.vel[0]
        msg.twist.linear.y     = e.vel[1]
        msg.twist.linear.z     = e.vel[2]
        msg.acc.x              = e.acc[0]
        msg.acc.y              = e.acc[1]
        msg.acc.z              = e.acc[2]
        msg.pose.orientation   = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, e.yaw))
        msg.twist.angular.x    = e.omega[0]
        msg.twist.angular.y    = e.omega[1]
        msg.twist.angular.z    = e.omega[2]
        print(msg)
        pub.publish(msg)
        rate.sleep()
