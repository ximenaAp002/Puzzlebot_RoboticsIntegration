#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
import math
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Int16, Bool, Float32
import random

t = 0
wmax = .2 
dt = 0.01
kpr = 1.4
kpt = 3.0

def posY_callback(data):
    global posY
    posY = data.pose.position.y

def posX_callback(data):
    global posX
    posX = data.pose.position.x

def ori_callback(data):
    global ori
    ori = data.data

def lidar_callback(data):
    global stop
    stop = data.data

def ruta():
    numPath = rospy.get_param("/ruta/numMesa")
    numPath = 1
    ruta_archivo = "/home/karma/gazebo/src/pathRestaurante/paths/path" + str(numPath) + "Try1.txt"

    datos = np.loadtxt(ruta_archivo, skiprows=1)

    posiciones_x = .01 * datos[:, 0]
    posiciones_y = .01 * datos[:, 1]

    diferenciaX = posiciones_x[0]
    diferenciaY = posiciones_y[0]

    for i in range(len(posiciones_x)):
        posiciones_x[i] -= diferenciaX
        posiciones_y[i] -= diferenciaY
    
    return  posiciones_x, posiciones_y

if __name__=="__main__":
    try:
        rospy.init_node('square_mover')

        nodeRate = 100
        rate = rospy.Rate(nodeRate)

        twist = Twist()

        posiciones_x, posiciones_y = ruta()

        posiciones_x_rev = np.flip(posiciones_x)
        posiciones_y_rev = np.flip(posiciones_y)

        num_positions = len(posiciones_x)

        error_dis_ant = 0.0
        error_ori_ant = 0.0

        posY = 0.0
        posX = 0.0
        ori = 0.0

        stop = False

        for k in range (2):
            pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

            orientacion = rospy.Subscriber('/z_Euler', Float32, ori_callback)
            pos_act_x = rospy.Subscriber('/slam_out_pose', PoseStamped, posX_callback)
            pos_act_y = rospy.Subscriber('/slam_out_pose', PoseStamped, posY_callback)

            rospy.Subscriber("/stop", Bool, lidar_callback)
            if stop:
                vmax = 0
            else: 
                vmax = 0.05

            i = 0
            reverse = False
            if k == 1:
                reverse = True
            while i < num_positions:
                if not reverse:                                                                                                           
                    x_des = posiciones_x[i]
                    y_des = posiciones_y[i]
                else:       
                    x_des = posiciones_x_rev[i]
                    y_des = posiciones_y_rev[i] 
                
                print("x: ", posX, "x deseada: ", x_des)
                print("y: ", posY, "y deseada: ", y_des)

                error_dis = math.sqrt((x_des-posX)**2 + (y_des-posY)**2)
                error_ori = math.atan2(y_des-posY, x_des-posY) - ori
                
                if error_ori > math.pi:
                    error_ori = error_ori - 2*math.pi
                elif error_ori < -math.pi:
                    error_ori = error_ori + 2*math.pi

                error_dis_der= error_dis - error_dis_ant
                error_ori_der = error_ori - error_ori_ant

                u_vel = kpr*error_dis + kpt*error_dis_der
                u_ori = kpr*error_ori + kpt*error_ori_der
                
                error_dis_ant = error_dis
                error_ori_ant = error_ori

                vr = u_vel+u_ori
                vl = u_vel-u_ori
                
                vref = vmax * (vr + vl)/2
                wref = wmax * (-kpr * error_ori)

                twist.linear.x = vref
                twist.angular.z = wref
                pub.publish(twist)

                if abs(error_dis) < 0.01:
                    i += 1
                t = t + dt
                rate.sleep()
            print("Ruta terminada ",k)
            twist.linear.x = 0
            twist.angular.z = 0
            pub.publish(twist)
            #if (pedido):
                #ACTIVAR NODO
                #HACER LOS PUBLISHERS
            rospy.sleep(10)
        print("mesa ", numPath )
    except rospy.ROSInterruptException:
        pass
