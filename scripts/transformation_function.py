#!/usr/bin/env python
import numpy as np
import math
class transformations:
	def __init__(self):
		pass
	def get_quaternion_from_euler(self,psi,phi,th):
		rot_matrix_1=np.array([[np.cos(psi),-np.sin(psi),0],[np.sin(psi),np.cos(psi),0],[0,0,1]])
		rot_matrix_2=np.array([[1,0,0],[0,np.cos(phi),-np.sin(phi)],[0,np.sin(phi),np.cos(phi)]])
		rot_matrix_3=np.array([[np.cos(th),0,np.sin(th)],[0,1,0],[-np.sin(th),0,np.cos(th)]])
		rot_matrix=np.matmul(np.matmul(rot_matrix_1,rot_matrix_2),rot_matrix_3)
		qw=((1+rot_matrix[0][0]+rot_matrix[1][1]+rot_matrix[2][2])**0.5)/2
		qx=(rot_matrix[2][1]-rot_matrix[1][2])/(4*qw)
		qy=(rot_matrix[0][2]-rot_matrix[2][0])/(4*qw)
		qz=(rot_matrix[1][0]-rot_matrix[0][1])/(4*qw)
		return [qx, qy, qz, qw]
	
	def get_eulerzxy_from_quaternion(self,quaternions):
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

	def get_rotmat_from_quaternion(self,quaternions):
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
		return(np.array([[r00,r01,r02],[r10,r11,r12],[r20,r21,r22]]))