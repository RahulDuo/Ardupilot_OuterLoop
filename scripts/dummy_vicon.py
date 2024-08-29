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
class pose_estimate:
	def __init__(self):
		self.vision_pose=PoseStamped()
		#print(vision_pose)
		self.communication_rate=100.0 #Hz
	def viconCb(self):
		self.vision_pose.header.stamp=rospy.Time.now()
		
		'''self.vision_pose.transform.translation.x=1
    	self.vision_pose.transform.translation.y=2
    	self.vision_pose.transform.translation.z=1
    	self.vision_pose.transform.rotation.x=0
    	self.vision_pose.transform.rotation.y=0
    	self.vision_pose.transform.rotation.z=0
    	self.vision_pose.transform.rotation.w=1
'''	
	def transform_from_euler(self,x, y, z, roll, pitch, yaw, header_frame_id,child_frame_id):
		t = TransformStamped()
		t.transform.translation.x = x
		t.transform.translation.y = y
		t.transform.translation.z = z

		q =[ 0, 0, 0, 1]#[0,0,0,1]#[ 0.2241439, -0.4829629, -0.1294095, 0.8365163 ]#[0,0,0,1]#transformations.quaternion_from_euler(roll, pitch, yaw)
		t.transform.rotation.x = q[0]
		t.transform.rotation.y = q[1]
		t.transform.rotation.z = q[2]
		t.transform.rotation.w = q[3]

		t.header.frame_id = header_frame_id
		t.child_frame_id = child_frame_id
		return t
def main():
	rospy.init_node('vision_pose_node',disable_signals=True, anonymous=True)                         # initiate node
	estimation=pose_estimate()
	rate=rospy.Rate(estimation.communication_rate)
	estimation_pub=rospy.Publisher('vicon/Rahul_D_1/Rahul_D_1',TransformStamped,queue_size=1)
	pub_data=estimation.transform_from_euler(0.0,0.0,0.5,0.0,0.0,0.0,'vicon','Rahul_D_1')
	while True:
		estimation_pub.publish(pub_data)
		print(pub_data)
		rate.sleep()

if __name__=='__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
