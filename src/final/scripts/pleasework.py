#!/usr/bin/env python
# --- Milestone 2 ---
import logging
import rospy
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from geometry_msgs.msg import PoseStamped,TransformStamped,PoseWithCovarianceStamped
from crazyflie_driver.msg import Position

drone_position = PoseStamped()
logging.basicConfig(filename='crash.log',level=logging.DEBUG)
def position_callback(msg):
    global drone_position
    drone_position = msg


def main():
    rospy.init_node('pleasework')

    buff = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buff)   
    
    rate = rospy.Rate(4)
    rate.sleep()    
    position = rospy.Subscriber('cf1/pose', PoseStamped, position_callback)    
    odo = None

    while not rospy.is_shutdown():
        try:
            if drone_position != None or drone_position.header.frame_id!='':
                #print("Original message")
                #print(drone_position)
                if(drone_position.header.frame_id!='map'):
                    if not buff.can_transform(drone_position.header.frame_id, 'map', drone_position.header.stamp, rospy.Duration(0.5)):
                        rospy.logwarn_throttle(5.0, 'No tranform from %s to map' % drone_position.header.frame_id)
                    else:
                        #print("buff.transform")
                        odo = buff.transform(drone_position, 'map')

                else:
                    print("ay")
                    odo = drone_position
                #if(odo.header.frame_id!='map'):
                
                print('Pose at time %s',odo.header.stamp)
                print(str(round(odo.pose.position.x, 2)))
                print(str(round(odo.pose.position.y, 2)))
                print(str(round(odo.pose.position.z, 2)))
                print("")


                logging.info('Pose at time %s',odo.header.stamp)
                logging.info(str(round(odo.pose.position.x, 2)))
                logging.info(str(round(odo.pose.position.y, 2)))
                logging.info(str(round(odo.pose.position.z, 2)))
                logging.info("")
            rate.sleep()
        except:
            pass

if __name__ == "__main__":
    main()
