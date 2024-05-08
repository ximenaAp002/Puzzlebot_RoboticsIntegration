#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

# Define callback functions for wheel velocities
def posY_callback(data):
    global posY
    posY = data.pose.position.x

def posX_callback(data):
    global posX
    posX = data.pose.position.x

def posZ_callback(data):
    global posZ
    posZ = data.pose.orientation.w
# Set up ROS node and publishers/subscribers

if __name__=="__main__":
    try:
        rospy.init_node('square_mover')

        nodeRate = 100
        rate = rospy.Rate(nodeRate)

        twist = Twist()
        vmax = .05

        # Define robot parameters
        wheel_radius = 0.05  # radius of wheels (m)
        wheelbase = 0.19  # distance between wheels (m) (l)
        dt = 0.01  # time step (s)
        t = 0  # Total time (s)
        v_max = 1  # maximum linear velocity (m/s)
        w_max = np.pi / 2  # maximum angular velocity (rad/s)
        
        # Read positions from file
        numPath = rospy.get_param('/ruta/numMesa')

        ruta_archivo = "/home/karma/gazebo/src/pathRestaurante/paths/path" + str(numPath) + "Try1.txt"

        datos = np.loadtxt(ruta_archivo, skiprows=1)

        posiciones_x = .01 * datos[:, 0]
        posiciones_y = .01 * datos[:, 1]

        diferenciaX = posiciones_x[0]
        diferenciaY = posiciones_y[0]

        for i in range(len(posiciones_x)):
            posiciones_x[i] -= diferenciaX
            posiciones_y[i] -= diferenciaY

        posiciones_x_rev = np.flip(posiciones_x)
        posiciones_y_rev = np.flip(posiciones_y)

        #print("Posiciones de x:", posiciones_x)
        #print("Posiciones de y:", posiciones_y)

        #print("Posiciones de x:", posiciones_x_rev)
        #print("Posiciones de y:", posiciones_y_rev)

        num_positions = len(posiciones_x)

        #pos_x = positions[]
        #print(positions[0])
        #rospy.sleep(1000)
        x = posiciones_x[0]  # x-position (m)
        y = posiciones_y[0] # y-position (m)


        kpr = 2.4
        kpt = 4.5

        error_dis_ant = 0.0
        error_ori_ant = 0.0


        posY = 0.0
        posX = 0.0
        posZ = 0.0

        # Simulate robot motion
        for k in range (2):
            pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

            orientacion = rospy.Subscriber('/slam_out_pose', Float32, posZ_callback)
            pos_act_x = rospy.Subscriber('/slam_out_pose', Float32, posX_callback)
            pos_act_y = rospy.Subscriber('/slam_out_pose', Float32, posY_callback)

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
                

                error_dis = math.sqrt((x_des-posX)^2 + (y_des-posY)^2)
                error_ori = math.atan2(y_des-posY, x_des-posY) - posZ
                
                if error_ori > math.pi:
                    error_ori = error_ori - 2*math.pi
                elif error_ori < -math.pi:
                    error_ori = error_ori + 2*math.pi

                
                error_dis_der = error_dis - error_dis_ant
                error_ori_der = error_ori - error_ori_ant

                u_vel = kpr*error_dis + kpt*error_dis_der
                u_ori = kpr*error_ori + kpt*error_ori_der
                
                error_dis_ant = error_dis
                error_ori_ant = error_ori
                
                #print('y actual = ', y, 'y deseada = ', yd)
                print(numPath)
                print('y deseada = ', y_des)
                print('x deseada = ', x_des)
                # print()
                #print('x actual = ', x_act, 'x deseada = ', xd)
                # print()
                # print('error =', error)
                # print()

                vr = u_vel+u_ori
                vl = u_vel-u_ori
                
                vref = (vr + vl)/2
                wref = -kpr * error_ori

                # Compute wheel velocities from desired linear and angular velocities
                twist.linear.x = vref
                twist.angular.z = wref
                pub.publish(twist)

                # Check if the robot has reached the desired position
                if abs(error_dis) < 0.01:
                    i += 1
                t = t + dt
                rate.sleep()
            print("Ruta terminada ",k)
            twist.linear.x = 0
            twist.angular.z = 0
            pub.publish(twist)
            rospy.sleep(10)
        print("mesa ", numPath )
    except rospy.ROSInterruptException:
        pass


