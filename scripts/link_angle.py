#!/usr/bin/env python
MODE="no-fly"
import time
#Import packages and classes
import rospy
from fcu_modes import *
#from controller_trajectory import *
from controller import *
#from controller_L_trajectry_contact import *


# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from geometry_msgs.msg import TransformStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Imu
import csv
import numpy as np
import math
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray

def get_eulerzxy_from_quaternion(quaternions):
    q0=quaternions.w
    q1=quaternions.x
    q2=quaternions.y
    q3=quaternions.z
    
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
    # 3x3 rotation matrix
    ##rot_matrix = np.array([[r00, r01, r02],
    ##                       [r10, r11, r12],
    ##                       [r20, r21, r22]])
    
    phi=math.atan2(r21,(r20**2+r22**2)**0.5)   #================minus?================ euler y
    theta=math.atan2(-r20/math.cos(phi),r22/math.cos(phi))      #doesn't work for phi=90 deg, 270 deg, highly unlikely, euler x
    psi=math.atan2(-r01/math.cos(phi),r11/math.cos(phi))             #doesn't work for phi=90 deg, 270 deg, highly unlikely, euler z
    return(np.array([[phi],[theta],[psi]]))





def linkangle_cb(msg):
        link_rot = Quaternion(0.0,0.0,0.0,1.0)
        link_rot.x=msg.transform.rotation.x
        link_rot.y=msg.transform.rotation.y
        link_rot.z=msg.transform.rotation.z
        link_rot.w=msg.transform.rotation.w
        #print(link_rot)
        link_rot_euler=get_eulerzxy_from_quaternion(link_rot)
        print(link_rot_euler[0][0]*180/math.pi,link_rot_euler[1][0]*180/math.pi,link_rot_euler[2][0]*180/math.pi)
        data =[link_rot_euler[0][0]*180/math.pi,link_rot_euler[1][0]*180/math.pi,link_rot_euler[2][0]*180/math.pi]
        with open("src/am_simulation/log_data/2024_06_22_04_l.csv", 'a') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(data)
            csvfile.close
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/vicon/Link_R1/Link_R1', TransformStamped, linkangle_cb)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
    control_loop_rate = 100
    rate = rospy.Rate(control_loop_rate)