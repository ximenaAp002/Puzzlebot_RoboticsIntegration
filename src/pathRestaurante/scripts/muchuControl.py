#!/usr/bin/env python

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int16, Bool

stop = False

def lidar_callback(data):
    global stop
    stop = data.data

def survey_callback(data):
    global score
    score = data.data

if __name__=="__main__":
    try:    
        rospy.init_node('mushu_control')
        nodeRate = 100
        rate = rospy.Rate(nodeRate)

        pubEmoji = rospy.Publisher("/mushuMode", Int16, queue_size=10)
        rospy.Subscriber("/stop", Bool, lidar_callback)
        rospy.Subscriber("/survey",String, survey_callback)
        while not rospy.is_shutdown():
            if stop:
                emoji = 3
                pubEmoji.publish(emoji)
                rospy.sleep(2)                
            elif not stop:
                emoji = 4
                pubEmoji.publish(emoji)
                rospy.sleep(2)
            elif score == "Anger":
                emoji = 5
                pubEmoji.publish(emoji)
                rospy.sleep(2)
            elif score == "Surprise":
                emoji = 4
                pubEmoji.publish(emoji)
                rospy.sleep(2)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass