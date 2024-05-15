#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

def callback(msg):
    # values at 0 degree
    #print("left ",msg.ranges[0])
    # values at 90 degree
    #print("front ",msg.ranges[360])
    frente = msg.ranges[360]
    if frente < .3:
        print("PARA")
        pub.publish(True)
    # values at 180 degree
    #print("right ",msg.ranges[719])

if __name__=="__main__":
    rospy.init_node('lidar')
    rospy.Subscriber('/scan', LaserScan, callback)
    pub = rospy.Publisher("/stop", Bool, queue_size=10)
    rospy.spin()