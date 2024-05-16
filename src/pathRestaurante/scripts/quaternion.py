#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped

def ori_callback(data):
    global oriW
    oriW = data.pose.orientation.w
def ori_callback(data):
    global oriZ
    oriZ = data.pose.orientation.z
def ori_callback(data):
    global oriY
    oriY = data.pose.orientation.y
def ori_callback(data):
    global oriX
    oriX = data.pose.orientation.x

def quaternion_to_euler():
    # Convertir el mensaje Quaternion a una lista
    quaternion_list = [oriX, oriY, oriZ , oriW]
    # Obtener los angulos de Euler a partir del quaternion
    roll, pitch, yaw = euler_from_quaternion(quaternion_list)
    return roll, pitch, yaw

def quaternion_callback(quaternion_msg):
    roll, pitch, yaw = quaternion_to_euler(quaternion_msg)
    print("Angulos de Euler (Roll, Pitch, Yaw):", roll, pitch, yaw)
    pubEuler.publish(yaw)

if __name__ == "__main__":
    rospy.init_node('eulerConverter')
    rospy.Subscriber("/slam_out_pose", PoseStamped, quaternion_callback)

    pubEuler = rospy.Publisher('/z_Euler', Float32, queue_size=10 )

    rospy.spin()    
