#!/usr/bin/env python
MODE="no-fly"
import time
#Import packages and classes
import rospy
from fcu_modes import *
#from controller_trajectory import *
#from controller import *
from controller_L_trajectry_contact import *


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

control_loop_rate=100
timestamp = rospy.Time()
from config import *

class Attitude:
	def __init__(self):
		self.att = AttitudeTarget()
		self.IMU_quat=Quaternion(0.0,0.0,0.0,0.0)
		self.thrust=0.0
		self.average_thrust=0.0
	
		

def main():
	rospy.init_node("mav_control_node")
	
	fcu=fcuModes()
	ol_cnt=outer_loop_controller()
	att_fun=Attitude()
	
	rospy.Subscriber('mavros/state', State,fcu.stateCb)
	rospy.Subscriber('/mavros/battery',BatteryState,fcu.batteryCb)
	rospy.Subscriber('mavros/imu/data', Imu, ol_cnt.ImuCb)
	rospy.Subscriber("/wall_reaction",Float64,ol_cnt.wall_reaction_Cb)
	rospy.Subscriber("/vicon/Link_R1/Link_R1",Float64,ol_cnt.linkangle_cb)
	#att_pub1=rospy.Publisher('/mavros/setpoint_position/local',PoseStamped,queue_size=1)
	att_pub=rospy.Publisher('mavros/setpoint_raw/attitude',AttitudeTarget,queue_size=1)
	
	rospy.Subscriber("/mavros/rc/in", RCIn,ol_cnt.RCCb)       

	rc_override = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=1)	# mode 0 = STABILIZE # mode 4 = GUIDED # mode 9 = LAND
	mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
	arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
	takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
	
	print('services started')
	rate = rospy.Rate(control_loop_rate)
	rate.sleep()
	fcu.setStabilizedMode(mode_service)
	
    # Data logging
	data=['Time','Target_x','Target_y','Target_z','current_x','current_y','current_z','error_x','error_y','error_z','df_x','df_y','df_z','error_int_x','error_int_y','error_int_z','rddot_x','rddot_y','rddot_z','Thrust_physical','thrust_command','phi_des','theta_des','yaw_des','quat_x_command','quat_y_command','quat_z_command','quat_w_command','quat_x_actual','quat_y_actual','quat_z_actual','quat_w_actual','phi_actual','theta_actual','psi_actual','IMU_phi','IMU_theta','IMU_psi','Wall_reaction']
	with open(FILE_NAME, 'w') as csvfile:                        
		csvwriter = csv.writer(csvfile)
		csvwriter.writerow(data)
		csvfile.close  


	if RUN_TYPE=="hardware":
		rospy.Subscriber('/vicon/Rahul_D_1/Rahul_D_1',TransformStamped, ol_cnt.viconCb1)            # Vicon pose subscriber for hardware experiment
	
		#rospy.Subscriber('/mavros/rc/in',RCIn, ol_cnt.RCCb)
	elif RUN_TYPE=="simulation":
		rospy.Subscriber('/gazebo/model_states',ModelStates, ol_cnt.GazeboCb1)
		
    

		
    
	t_wait=rospy.get_time()
	while not ol_cnt.vicon_publishing==False:
		print("Waiting for vicon signal")
		if rospy.get_time()-t_wait>10.0:
			print("\n\n\n####################################\n   VICON DATA NOT RECEIVED in 10s\n####################################\n\n\n")
			raise KeyboardInterrupt
			break
		rate.sleep()

	print(fcu.state.armed)

	k=0
	while k<10:
		att=ol_cnt.att
		att.thrust=0.2
		fcu.att_publish(att_pub,att)
		rate.sleep()
		k = k + 1
		
	while fcu.state.armed!=True:
		print(fcu.state.armed)
		arm_service(True)
		rate.sleep()
		
	print(fcu.state.armed)
	time.sleep(0.1)
	
	fcu.GuidedNonGPSMode(mode_service)
	
	print('Hello OffboardStarted')
	
	prev_time=rospy.get_time()
	init_time=prev_time
	t_init=rospy.get_time()
	# t_total_H=ol_cnt.t_takeoff+ol_cnt.t_positioning+ol_cnt.t_helix+ol_cnt.t_positioning+ol_cnt.t_takeoff
	t_total_L =ol_cnt.TIME_TAKEOFF+ol_cnt.TIME_HOVER+ol_cnt.TIME_FORWARD+ol_cnt.t1+ol_cnt.t2+ol_cnt.t3+ol_cnt.t4+ol_cnt.TIME_BACKWARD+ol_cnt.TIME_LAND
       	
	try:
		
		while rospy.get_time()-t_init< t_total_L and rospy.get_time()-ol_cnt.prev_time<2.0:
			trajectory_time=rospy.get_time()-t_init
			#print(trajectory_time)
			ol_cnt.trajectory(trajectory_time)
			ol_cnt.outerloop1()	
			att_pub.publish(ol_cnt.att)
			rate.sleep()

		"""
		while rospy.get_time()- ol_cnt.prev_time<2.0:
			#if ol_cnt.RC_values[6]<1400:
			ol_cnt.outerloop1()
			print("outerloop")
			att_pub.publish(ol_cnt.att)
			rate.sleep()
		"""
		print("\n\n\n#############################\n   VICON DATA TIMEOUT \n#############################\n\n\n")
		print('disarming')
		fcu.land(mode_service)
		fcu.disarm(arm_service)
		
	except KeyboardInterrupt:
		while True:
			ol_cnt.emergency_land()
			att_pub.publish(ol_cnt.att)
            #print(cnt.att)
			rate.sleep()

if __name__=="__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass
