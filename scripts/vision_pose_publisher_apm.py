#!/usr/bin/env python
# ROS python API
import rospy
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Pose
from geometry_msgs.msg import TransformStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
import csv
import numpy as np
import math
import transformation_function

class pose_estimate:
	def __init__(self):
		self.tF=transformation_function.transformations()
		self.vision_pose=PoseStamped()
		self.vision_pose.header.stamp=rospy.Time.now()
		self.vision_pose.header.frame_id="world"
		self.vision_pose.pose.position.x=0
		self.vision_pose.pose.position.y=0
		self.vision_pose.pose.position.z=0
		self.vision_pose.pose.orientation.x=0.0
		self.vision_pose.pose.orientation.y=0.0
		self.vision_pose.pose.orientation.z=0.0
		self.vision_pose.pose.orientation.w=1.0

		self.communication_rate=80.0 #Hz
		self.vicon_rot_euler=[0,0,0]
	def visionCb(self,msg):
		self.vision_pose.header.stamp=rospy.Time.now()
		self.vision_pose.pose.position.x=round(msg.transform.translation.x,3)
		self.vision_pose.pose.position.y=round(msg.transform.translation.y,3)
		self.vision_pose.pose.position.z=round(msg.transform.translation.z,3)
		self.vicon_rot_euler=self.tF.get_eulerzxy_from_quaternion(msg.transform.rotation)
		quat_cur=self.tF.get_quaternion_from_euler(self.vicon_rot_euler[2][0]+0.0*math.pi/2,self.vicon_rot_euler[0][0],self.vicon_rot_euler[1][0]);
		self.vision_pose.pose.orientation.x=quat_cur[0]#round(msg.transform.rotation.x,4)
		self.vision_pose.pose.orientation.y=quat_cur[1]#round(msg.transform.rotation.y,4)
		self.vision_pose.pose.orientation.z=quat_cur[2]#round(msg.transform.rotation.z,4)
		self.vision_pose.pose.orientation.w=quat_cur[3]#round(msg.transform.rotation.w,4)

def main():
	rospy.init_node('vision_pose_node',disable_signals=True, anonymous=True)                         # initiate node
	estimation=pose_estimate()
	rate=rospy.Rate(estimation.communication_rate)
	#estimation_pub=rospy.Publisher('mavros/vision_pose/pose',PoseStamped,queue_size=1)
	
	estimation_pub=rospy.Publisher('mavros/mocap/pose',PoseStamped,queue_size=1)
	rospy.Subscriber('vicon/Rahul_D_1/Rahul_D_1',TransformStamped, estimation.visionCb)
	while True:
		estimation_pub.publish(estimation.vision_pose)
		print(estimation.vision_pose)
		print(estimation.vicon_rot_euler[2]*180/math.pi+math.pi/2)
		rate.sleep()

if __name__=='__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
