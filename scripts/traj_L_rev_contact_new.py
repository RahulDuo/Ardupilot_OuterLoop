import math
import csv
from config import *
from geometry_msgs.msg import Point,Pose, PoseStamped, Twist, Quaternion
class traj_gen():
	def __init__(self):
		self.trans_pos_des=Point(0.0,0.0,0.0)
		self.trans_vel_des=Point(0.0,0.0,0.0)
		self.trans_acc_des=Point(0.0,0.0,0.0)
		self.rot_pos_des=Point(0.0,0.0,0.0)
		self.intergrator_on=0
		self.contact_mode=False


		self.t1=20  #make contact with the wall
		self.t2=10 #left-up vertical
		self.t3=10  #left-down vertical
		self.t4=5  #centre horizotal
		
		self.error_y=0.05;
		self.H_vert=(HOVER_POS_FINAL.z - HOVER_POS.z)/2
		self.H_horz=0#0.3/2
	def trajectory_matrix(self):
		self.M=np.array([[1,self.t0,self.t0**2,self.t0**3,self.t0**4,self.t0**5],[0,1,2*self.t0,3*self.t0**2,4*self.t0**3,5*self.t0**4],[0,0,2,6*self.t0,12*self.t0**2,20*self.t0**3],[1,self.tf,self.tf**2,self.tf**3,self.tf**4,self.tf**5],[0,1,2*self.tf,3*self.tf**2,4*self.tf**3,5*self.tf**4],[0,0,2,6*self.tf,12*self.tf**2,20*self.tf**3]])
	def traj_data(self,t,fcu):

		csvreader=csv.reader(open('src/ol/setpoint_files/setpoint_1.csv'))
		for row in csvreader:
			self.intergrator_on=int(row[0])
		#Take-off and reach hover-point
		if t<=TIME_TAKEOFF:
			self.contact_mode=False
			self.t0=0
			self.tf=TIME_TAKEOFF
			self.trajectory_matrix()
			b=np.array([[INITIAL_POS.x,INITIAL_POS.y,INITIAL_POS.z],[0,0,0],[0,0,0],[HOVER_POS.x,HOVER_POS.y,HOVER_POS.z],[0,0,0],[0,0,0]])
		# Hovering for the calibration
		elif t>TIME_TAKEOFF and t<=TIME_TAKEOFF+TIME_HOVER:
			self.contact_mode=False
			self.t0=TIME_TAKEOFF
			self.tf=TIME_TAKEOFF+TIME_HOVER
			self.trajectory_matrix()
			b=np.array([[HOVER_POS.x,HOVER_POS.y,HOVER_POS.z],[0,0,0],[0,0,0],[HOVER_POS.x,HOVER_POS.y,HOVER_POS.z],[0,0,0],[0,0,0]])			
		#Approaching the wall
		elif t>TIME_TAKEOFF+TIME_HOVER and t<=TIME_TAKEOFF+TIME_HOVER+TIME_FORWARD:
			self.contact_mode=False
			self.t0=TIME_TAKEOFF+TIME_HOVER
			self.tf=TIME_TAKEOFF+TIME_HOVER+TIME_FORWARD
			self.trajectory_matrix()
			b=np.array([[HOVER_POS.x,HOVER_POS.y,HOVER_POS.z],[0,0,0],[0,0,0],[HOVER_POS.x,CONTACT_POS.y,HOVER_POS.z],[0,0,0],[0,0,0]])
		#make contact with the wall
		elif t>TIME_TAKEOFF+TIME_FORWARD+TIME_HOVER and t<=TIME_TAKEOFF+TIME_FORWARD+TIME_HOVER+self.t1:
			self.contact_mode=True
			self.t0=TIME_TAKEOFF+TIME_FORWARD+TIME_HOVER
			self.tf=TIME_TAKEOFF+TIME_FORWARD+TIME_HOVER+self.t1
			self.trajectory_matrix()
			b=np.array([[HOVER_POS.x,CONTACT_POS.y,HOVER_POS.z],[0,0,0],[0,0,0],[HOVER_POS.x,CONTACT_POS.y+1.0*self.error_y,HOVER_POS.z],[0,0,0],[0,0,0]])
			b1=np.array([[0,0,0],[0,0,0],[0,0,0],[PHI_TILT,0,0],[0,0,0],[0,0,0]])
		#Maintaining the contact without moving
		elif t>TIME_TAKEOFF+TIME_FORWARD+TIME_HOVER+self.t1 and t<=TIME_TAKEOFF+TIME_FORWARD+TIME_HOVER+self.t1+self.t2:
			self.contact_mode=True
			self.t0=TIME_TAKEOFF+TIME_FORWARD+TIME_HOVER+self.t1
			self.tf=TIME_TAKEOFF+TIME_FORWARD+TIME_HOVER+self.t1+self.t2
			self.trajectory_matrix()
			b=np.array([[HOVER_POS.x,CONTACT_POS.y+self.error_y,HOVER_POS.z],[0,0,0],[0,0,0],[HOVER_POS.x+self.H_horz,CONTACT_POS.y+self.error_y,HOVER_POS.z],[0,0,0],[0,0,0]])
			b1=np.array([[PHI_TILT,0,0],[0,0,0],[0,0,0],[PHI_TILT,0,0],[0,0,0],[0,0,0]])		
		#Move along the surface while maintaining the contact
		elif t>TIME_TAKEOFF+TIME_FORWARD+TIME_HOVER+self.t1+self.t2 and t<=TIME_TAKEOFF+TIME_FORWARD+TIME_HOVER+self.t1+self.t2+self.t3:
			self.contact_mode=True
			self.t0=TIME_TAKEOFF+TIME_FORWARD+TIME_HOVER+self.t1+self.t2
			self.tf=TIME_TAKEOFF+TIME_FORWARD+TIME_HOVER+self.t1+self.t2+self.t3
			self.trajectory_matrix()
			b=np.array([[HOVER_POS.x,CONTACT_POS.y+self.error_y,HOVER_POS.z],[0,0,0],[0,0,0],[HOVER_POS.x+self.H_horz,CONTACT_POS.y+self.error_y,HOVER_POS_FINAL.z],[0,0,0],[0,0,0]])
			b1=np.array([[PHI_TILT,0,0],[0,0,0],[0,0,0],[PHI_TILT,0,0],[0,0,0],[0,0,0]])
		#remove contact
		elif t>TIME_TAKEOFF+TIME_FORWARD+TIME_HOVER+self.t1+self.t2+self.t3 and t<=TIME_TAKEOFF+TIME_FORWARD+TIME_HOVER+self.t1+self.t2+self.t3+self.t4:
			self.contact_mode=True
			self.t0=TIME_TAKEOFF+TIME_FORWARD+TIME_HOVER+self.t1+self.t2+self.t3
			self.tf=TIME_TAKEOFF+TIME_FORWARD+TIME_HOVER+self.t1+self.t2+self.t3+self.t4
			self.trajectory_matrix()
			b=np.array([[HOVER_POS.x+self.H_horz,CONTACT_POS.y+self.error_y,HOVER_POS_FINAL.z],[0,0,0],[0,0,0],[HOVER_POS.x+self.H_horz,CONTACT_POS.y,HOVER_POS_FINAL.z],[0,0,0],[0,0,0]])
			b1=np.array([[PHI_TILT,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]])
		elif t>TIME_TAKEOFF+TIME_FORWARD+TIME_HOVER+self.t1+self.t2+self.t3+self.t4 and t<=TIME_TAKEOFF+TIME_FORWARD+TIME_HOVER+self.t1+self.t2+self.t3+self.t4+TIME_BACKWARD:
			self.contact_mode=False
			self.t0=TIME_TAKEOFF+TIME_FORWARD+TIME_HOVER+self.t1+self.t2+self.t3+self.t4
			self.tf=TIME_TAKEOFF+TIME_FORWARD+TIME_HOVER+self.t1+self.t2+self.t3+self.t4+TIME_BACKWARD
			self.trajectory_matrix()
			b=np.array([[HOVER_POS.x+self.H_horz,CONTACT_POS.y,HOVER_POS_FINAL.z],[0,0,0],[0,0,0],[HOVER_POS.x,HOVER_POS.y,HOVER_POS_FINAL.z],[0,0,0],[0,0,0]])
		elif t>TIME_TAKEOFF+TIME_FORWARD+TIME_HOVER+self.t1+self.t2+self.t3+self.t4+TIME_BACKWARD and t<=TIME_TAKEOFF+TIME_FORWARD+TIME_HOVER+self.t1+self.t2+self.t3+self.t4+TIME_BACKWARD+TIME_LAND:
			self.contact_mode=False
			self.t0=TIME_TAKEOFF+TIME_FORWARD+TIME_HOVER+self.t1+self.t2+self.t3+self.t4+TIME_BACKWARD
			self.tf=TIME_TAKEOFF+TIME_FORWARD+TIME_HOVER+self.t1+self.t2+self.t3+self.t4+TIME_BACKWARD+TIME_LAND
			self.trajectory_matrix()
			b=np.array([[HOVER_POS.x,HOVER_POS.y,HOVER_POS_FINAL.z],[0,0,0],[0,0,0],[INITIAL_POS.x,INITIAL_POS.y,INITIAL_POS.z],[0,0,0],[0,0,0]])
		if self.contact_mode==False:
			self.rot_pos_des=Point(0.0,0.0,0.0)
		else:
			a1=np.linalg.solve(self.M,b1)
			out= np.array([a1[0] + a1[1]*t + a1[2]*t**2 + a1[3]*t**3 + a1[4]*t**4 + a1[5]*t**5])
			if abs(out[0][0])<=abs(PHI_TILT):
				self.rot_pos_des.x=out[0][0]
		a=np.linalg.solve(self.M,b)	
		out= np.array([a[0] + a[1]*t + a[2]*t**2 + a[3]*t**3 + a[4]*t**4 + a[5]*t**5])
		outd= np.array([a[1] + 2*a[2]*t + 3*a[3]*t**2 + 4*a[4]*t**3 + 5*a[5]*t**4])
		outdd= np.array([2*a[2] + 6*a[3]*t + 12*a[4]*t**2 + 20*a[5]*t**3])
		self.trans_pos_des=Point(out[0][0],out[0][1],out[0][2])
		self.trans_vel_des=Point(outd[0][0],outd[0][1],outd[0][2])
		self.trans_acc_des=Point(outdd[0][0],outdd[0][1],outdd[0][2])
		return [self.intergrator_on,self.trans_pos_des,self.trans_vel_des,self.trans_acc_des,self.rot_pos_des,self.contact_mode,t]