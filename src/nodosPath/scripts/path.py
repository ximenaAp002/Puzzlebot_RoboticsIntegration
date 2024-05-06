#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

# Define callback functions for wheel velocities
def wl_callback(data):
    global wl
    wl = data.data

def wr_callback(data):
    global wr
    wr = data.data

def lidar_callback(data):
    global lidar
    lidar = data.data
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
        numPath = rospy.get_param('/mesa/numMesa')

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
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        sub_wl = rospy.Subscriber('/wl', Float32, wl_callback)
        sub_wr = rospy.Subscriber('/wr', Float32, wr_callback)

        x = posiciones_x[0]  # x-position (m)
        y = posiciones_y[0] # y-position (m)
        theta = 0  # orientation (rad)

        kpr = 2.4
        kpt = 4.5

        wr = 0.0
        wl = 0.0

        # Simulate robot motion
        for k in range (2):
            sub_lidar = rospy.Subscriber('/scan', Float32, lidar_callback)

            
            i = 0
            reverse = False
            if k == 1:
                reverse = True
            while i < num_positions:
                if not reverse:
                    xd = posiciones_x[i]
                    yd = posiciones_y[i]
                else:       
                    xd = posiciones_x_rev[i]
                    yd = posiciones_y_rev[i]                   

                thetad = math.atan2((yd-y), (xd-x))
                error = math.sqrt((xd-x)**2 + (yd-y)**2)

                thetae = (theta - thetad)
                if thetae > math.pi:
                    thetae = thetae - 2*math.pi
                elif thetae < -math.pi:
                    thetae = thetae + 2*math.pi

                wref = -kpr * thetae
                vref = vmax*math.tanh(error*kpt/vmax)

                vr = vref + (wheelbase*wref)/2
                vl = vref - (wheelbase*wref)/2
                
                vref = (vr + vl)/2

                v_real = wheel_radius* (wr+wl)/2
                w_real = wheel_radius* (wr-wl)/wheelbase

                vx = v_real * math.cos(theta)
                vy = v_real * math.sin(theta)

                x = x + vx*dt
                y = y + vy*dt
                theta = theta + w_real * dt
                
                #print('y actual = ', y, 'y deseada = ', yd)
                print(numPath)
                print('y deseada = ', yd)
                print('x deseada = ', xd)
                # print()
                #print('x actual = ', x, 'x deseada = ', xd)
                # print()
                # print('error =', error)
                # print()

                # Compute wheel velocities from desired linear and angular velocities
                twist.linear.x = vref
                twist.angular.z = wref
                pub.publish(twist)

                # Check if the robot has reached the desired position
                if abs(error) < 0.01:
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


