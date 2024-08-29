#!/usr/bin/env python
# ROS python API
import rospy

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point,Pose, PoseStamped, Twist, Quaternion
from geometry_msgs.msg import TransformStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import Imu
from sensor_msgs.msg import BatteryState
import csv
import numpy as np
import math

import transformation_function
from config import *
class fcuModes:
	def __init__(self):
		self.control_loop_rate=100
		self.state = State()
		self.rc = RCIn()
		self.pose = Pose()
		self.att = AttitudeTarget()
		self.timestamp = rospy.Time()

		self.tF=transformation_function.transformations()
		self.battery_voltage_total=0.0

	def stateCb(self, msg):
		self.state = msg

	def batteryCb(self,msg):
		self.battery_voltage_total=round(msg.voltage,3)
		self.battery_voltage_cells=msg.cell_voltage
		self.battery_current=round(msg.current,3)

	def arm(arm_service):
		"""
		Arm the throttle
		"""
		return arm_service(True)
	
	def disarm(arm_service):
		"""
		Disarm the throttle
		"""
		return arm_service(False) 
	
	def setStabilizedMode(self,mode_service):
		mode_resp = mode_service(custom_mode="0")
		return mode_resp 
	
	def GuidedNonGPSMode(self,mode_service):
		mode_resp = mode_service(custom_mode="20")
		return mode_resp     
	       
	def land(self,mode_service):
		"""
		Set in LAND mode, which should cause the UAV to descend directly,
		land, and disarm.
		"""
		resp = mode_service(custom_mode="9")
		#self.disarm()

	def emergency_land(self,att_pub,average_thrust):
		quats=self.tF.get_quaternion_from_euler(YAW_DES_DEG*math.pi/180,0,0)
		self.att.orientation.x=quats[0]
		self.att.orientation.y=quats[1]
		self.att.orientation.z=quats[2]
		self.att.orientation.w=quats[3]
		self.att.thrust=average_thrust*0.9
		att_pub.publish(self.att)


	def att_publish(self,att_pub,att_des):
		
		self.att.orientation.x=att_des.orientation.x#self.ol_cnt.att.orientation.x
		self.att.orientation.y=att_des.orientation.y#self.ol_cnt.att.orientation.y
		self.att.orientation.z=att_des.orientation.z#self.ol_cnt.att.orientation.z
		self.att.orientation.w=att_des.orientation.w#self.ol_cnt.att.orientation.w
		self.att.thrust=att_des.thrust
		
		att_pub.publish(self.att)
		