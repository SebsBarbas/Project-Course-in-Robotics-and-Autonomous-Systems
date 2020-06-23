#!/usr/bin/env python
import rospy
import math
from crazyflie_driver.msg import Position
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

cmd_pos = Position()
got_pos = False
pprint = False

def cb(msg):
    global cmd_pos, got_pos, pprint
    cmd_pos.header.frame_id = msg.header.frame_id
    cmd_pos.header.stamp = msg.header.stamp
    cmd_pos.x = msg.pose.position.x
    cmd_pos.y = msg.pose.position.y
    cmd_pos.z = msg.pose.position.z
    _, _, temp_yaw = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
    cmd_pos.yaw = math.degrees(temp_yaw)
    got_pos = True    
    pprint = True

pub = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
sub = rospy.Subscriber('/cf1/hold_pos', PoseStamped, cb)

def main():
    global got_pos, pprint
    print("broadcast is running (on a diet)")
    rospy.init_node("diet_broadcast")
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        if got_pos:
            pub.publish(cmd_pos)
            if pprint:
                print("")
                print(cmd_pos)
                pprint = False
        rate.sleep()  # Remember to sleep kids

if __name__ == '__main__':
    main()