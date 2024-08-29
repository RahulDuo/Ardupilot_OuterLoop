# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point,Pose, PoseStamped, Twist, Quaternion
from geometry_msgs.msg import TransformStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
#from filterData import *

import transformation_function
import rospy
import math
import time
import csv
import numpy as np
from config import *


class outer_loop_controller:
	def __init__(self):
		self.tF=transformation_function.transformations()
		#self.flt=filterData()

		#manipulator
		"""
		self.trans_pos_link_2=Point(0.0,0.0,0.0)
		self.rot_quat_link_2=Quaternion(0.0,0.0,0.0,1.0)
		self.vicon_rot_euler_link_2=[[0],[0],[0]]
		"""

		self.trans_pos=Point(0.0,0.0,0.0)
		self.trans_pos_prev=Point(0.0,0.0,0.0)
		self.trans_pos_filt=Point(0.0,0.0,0.0)
		self.trans_pos_filt_prev=Point(0.0,0.0,0.0)
		self.trans_pos_count=Point(0.0,0.0,0.0)

		""""""
		""""""
		self.IMU_rot_euler = np.array([[0.0],[0.0],[0.0]])
		self.position_target=Point()
		self.IMU_quat=Quaternion(0.0,0.0,0.0,0.0)
		# Data from vicon
		self.vicon_pos=Point(0.0,0.0,0.0)
		self.vicon_pos_prev=Point(0.0,0.0,0.0)
		self.vicon_pos_prev_prev=Point(0.0,0.0,10.0)        
		self.vicon_rot=Quaternion(0.0,0.0,0.0,1.0)
		self.link_rot = Quaternion(0.0,0.0,0.0,1.0)
		self.link_rot_euler = np.array([[0.0],[0.0],[0.0]])
		self.prev_time_controller=0.0
		self.pos_error=np.array([[0.0],[0.0],[0.0]])
		self.pos_error_prev=np.array([[0.0],[0.0],[0.0]])
		self.error_integral=np.array([[0.0],[0.0],[0.0]])
		self.vicon_frequency=125.0
		self.df=np.array([[0.0],[0.0],[0.0]])                                       #the new filter 
		self.df_prev=np.array([[0.0],[0.0],[0.0]])
		self.tau=0.0333 #50Hz low-pass filter
        
		self.pos_error_prev_prev=np.array([[0.0],[0.0],[0.0]])
		self.UAV_rot_mat=np.array([[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]])
		
		self.integral_error_limit=6
		self.integral_error_influence_dist=1.5#m
		self.floor_elevation=0.4
		self.control_loop_rate = 100
		self.RC_values=[0.0]*8

		self.att = AttitudeTarget()
		self.att.orientation.x=0.0
		self.att.orientation.y=0.0
		self.att.orientation.z=0.0
		self.att.orientation.w=1.0
		self.att.thrust=0.1
		self.thrust_physical=0.0
		self.scale=1

		self.g=9.81 #m/s^2
		self.mass = 2.0  # kg
		self.ge3=np.array([[0.0],[0.0],[self.g]])
		self.rdes_ddot=np.array([[0.0],[0.0],[0.0]])
		self.phi_des=0.0
		self.yaw_des=0 #math.pi/2
		self.theta_des=0.0
		self.yaw_offset =0.0
		self.prev_time=rospy.get_time()
		self.prev_prev_time=rospy.get_time()
		

		self.battery_voltage_total=0.0
		self.battery_voltage_cells=[0.0,0.0,0.0,0.0]
		self.battery_current=0.0

		self.average_thrust=0.0
		self.average_thrust_count=0
		self.vicon_publishing=False
		self.intergrator_on=0

			# Wall Reaction 

		self.wall_reaction=0.0

		self.x_max=3 #m
		self.y_max=3 #m
		self.z_max=3 #m
		self.flag_wall=False

		if RUN_TYPE=="hardware":
			self.kp=np.array([[1.60,0,0],[0,1.2,0],[0,0,3.2]])
			self.kv=np.array([[3.2,0,0],[0,3.0,0],[0,0,0.75*6]])
			self.ki=np.array([[0.5*1.5,0,0],[0,0.5*1.2,0],[0,0,1.8*0.5]])

		elif RUN_TYPE=="simulation":
            #pass
			self.kp=np.array([[3,0,0],[0,3,0],[0,0,4.5]])
			self.kv=np.array([[2.25,0,0],[0,2.25,0],[0,0,5]])
			self.ki=np.array([[0.3,0,0],[0,0.3,0],[0,0,0.35]])
        

		#for gazebo latency
		
		self.vicon_latency=15 #ms (-5 for no delay)
		self.gazebo_location_history=[[0.0,0.0,0.0]]*int(self.vicon_latency//4.0+2)
		self.tele_latency=15 #ms (-13 for no delay)
		self.tele_comand_history=[[0.0,0.0,0.0,0.0,0.0]]*int(self.tele_latency//(1000.0/self.control_loop_rate)+2)
		self.gazebo_std_dev_xy=0.0 #mm
		self.gazebo_std_dev_z=0.0 #mm
		self.gazebo_vicon_offset=0.0 #m
		self.gazebo_std_dev_angular=0.0 #deg
        #for gazebo latency ends
    
		
		""""""
		""""""
		
		self.rot_pos=Point(0.0,0.0,0.0)
		self.rot_pos_prev=Point(0.0,0.0,0.0)
		self.rot_pos_filt=Point(0.0,0.0,0.0)
		self.rot_pos_filt_prev=Point(0.0,0.0,0.0)
		self.rot_pos_count=Point(0.0,0.0,0.0)

		self.trans_vel=Point(0.0,0.0,0.0)
		self.trans_vel_prev=Point(0.0,0.0,0.0)
		self.trans_vel_prev_prev=Point(0.0,0.0,0.0)
		self.trans_vel_filt=Point(0.0,0.0,0.0)
		self.trans_vel_filt_prev=Point(0.0,0.0,0.0)
		self.trans_vel_filt_prev_prev=Point(0.0,0.0,0.0)
		self.trans_vel_count=Point(0.0,0.0,0.0)

		self.rot_vel=Point(0.0,0.0,0.0)
		self.rot_vel_prev=Point(0.0,0.0,0.0)
		self.rot_vel_prev_prev=Point(0.0,0.0,0.0)
		self.rot_vel_filt=Point(0.0,0.0,0.0)
		self.rot_vel_filt_prev=Point(0.0,0.0,0.0)
		self.rot_vel_filt_prev_prev=Point(0.0,0.0,0.0)
		self.rot_vel_count=Point(0.0,0.0,0.0)

		self.rot_quat=Quaternion(0.0,0.0,0.0,1.0) 

		self.trans_pos_error=np.array([0.0,0.0,0.0])
		self.trans_vel_error=np.array([0.0,0.0,0.0])
		self.trans_pos_error_integral=np.array([0.0,0.0,0.0])

		

		self.trans_pos_des=Point(0.0,0.0,0.0)
		self.trans_vel_des=Point(0.0,0.0,0.0)
		self.trans_acc_des=Point(0.0,0.0,0.0)
		
		

		"""
		self.wall_reaction=0.0
		self.theta_1_des=0.0
		self.theta_1=0.0
		self.theta_2=0.0
		self.tau_1=0.0
		"""

	def viconCb(self,msg):
		self.vicon_publishing=True
		self.trans_pos.x=msg.transform.translation.x
		self.trans_pos.y=msg.transform.translation.y
		self.trans_pos.z=msg.transform.translation.z
		self.rot_quat.x=msg.transform.rotation.x
		self.rot_quat.y=msg.transform.rotation.y
		self.rot_quat.z=msg.transform.rotation.z
		self.rot_quat.w=msg.transform.rotation.w

		self.vicon_rot_euler=self.tF.get_eulerzxy_from_quaternion(self.rot_quat)
		self.rot_pos.z=self.vicon_rot_euler[2][0]
		self.UAV_rot_mat=self.tF.get_rotmat_from_quaternion(self.rot_quat)

		self.dt=rospy.get_time()-self.prev_time
		self.prev_time=rospy.get_time()
		#self.filter_states()
	
	def viconCb1(self,msg):
        #print(msg)
		self.vicon_publishing=True
		self.vicon_pos.x=msg.transform.translation.x
		self.vicon_pos.y=msg.transform.translation.y
		self.vicon_pos.z=msg.transform.translation.z
		self.vicon_rot.x=msg.transform.rotation.x
		self.vicon_rot.y=msg.transform.rotation.y
		self.vicon_rot.z=msg.transform.rotation.z
		self.vicon_rot.w=msg.transform.rotation.w
        #print("vicon location x,y,z: {}, {}, {}".format(self.vicon_pos.x,self.vicon_pos.y,self.vicon_pos.z))
		self.vicon_rot_euler=self.tF.get_eulerzxy_from_quaternion(self.vicon_rot)
		self.UAV_rot_mat=self.tF.get_rotmat_from_quaternion(self.vicon_rot)
        # print("UAV Roll Pitch Yaw: {} {} {}".format(round(self.vicon_rot_euler[0][0]*180/math.pi,2),round(self.vicon_rot_euler[1][0]*180/math.pi,2),round(self.vicon_rot_euler[2][0]*180/math.pi,2)))
		self.errors_calculation1()

		
	def GazeboCb(self,msg):
		self.vicon_publishing=True
		self.rot_quat.x=msg.pose[1].orientation.x
		self.rot_quat.y=msg.pose[1].orientation.y
		self.rot_quat.z=msg.pose[1].orientation.z
		self.rot_quat.w=msg.pose[1].orientation.w
		#print("Vicon location x,y,z: {}, {}, {}".format(self.vicon_pos.x,self.vicon_pos.y,self.vicon_pos.z))
		self.vicon_rot_euler=self.tF.get_eulerzxy_from_quaternion(self.rot_quat)
		self.rot_pos.z=self.vicon_rot_euler[2][0]
		self.UAV_rot_mat=self.tF.get_rotmat_from_quaternion(self.rot_quat)
		
		#self.gazebo_location_history.insert(0,[msg.pose[1].position.x+self.gazebo_vicon_offset*math.sin(self.vicon_rot_euler[1][0]+np.random.normal(0,self.gazebo_std_dev_angular)*math.pi/180)+np.random.normal(0,self.gazebo_std_dev_xy/1000),msg.pose[1].position.y-self.gazebo_vicon_offset*math.sin(self.vicon_rot_euler[0][0]+np.random.normal(0,self.gazebo_std_dev_angular)*math.pi/180)+np.random.normal(0,self.gazebo_std_dev_xy/1000),msg.pose[1].position.z+self.gazebo_vicon_offset+np.random.normal(0,self.gazebo_std_dev_z/1000)])
		self.trans_pos.x=msg.pose[1].position.x#self.gazebo_location_history[-1][0]
		self.trans_pos.y=msg.pose[1].position.y#self.gazebo_location_history[-1][1]
		self.trans_pos.z=msg.pose[1].position.z#self.gazebo_location_history[-1][2]
		self.dt=rospy.get_time()-self.prev_time
		self.prev_time=rospy.get_time()
		self.filter_states()
		self.RCCb_simulation()
	def GazeboCb1(self,msg):
		self.vicon_publishing=True
		self.vicon_rot.x=msg.pose[1].orientation.x
		self.vicon_rot.y=msg.pose[1].orientation.y
		self.vicon_rot.z=msg.pose[1].orientation.z
		self.vicon_rot.w=msg.pose[1].orientation.w
        # sprint(self.vicon_rot)
        #print("vicon location x,y,z: {}, {}, {}".format(self.vicon_pos.x,self.vicon_pos.y,self.vicon_pos.z))
		self.vicon_rot_euler=self.tF.get_eulerzxy_from_quaternion(self.vicon_rot)
		self.UAV_rot_mat=self.tF.get_rotmat_from_quaternion(self.vicon_rot)
        
        # self.gazebo_location_history.insert(0,[msg.pose[1].position.x+self.gazebo_vicon_offset*math.sin(self.vicon_rot_euler[1][0]+np.random.normal(0,self.gazebo_std_dev_angular)*math.pi/180)+np.random.normal(0,self.gazebo_std_dev_xy/1000),msg.pose[1].position.y-self.gazebo_vicon_offset*math.sin(self.vicon_rot_euler[0][0]+np.random.normal(0,self.gazebo_std_dev_angular)*math.pi/180)+np.random.normal(0,self.gazebo_std_dev_xy/1000),msg.pose[1].position.z+self.gazebo_vicon_offset+np.random.normal(0,self.gazebo_std_dev_z/1000)])
		self.gazebo_location_history.insert(0,[msg.pose[1].position.x,msg.pose[1].position.y,msg.pose[1].position.z])
		self.vicon_pos.x=self.gazebo_location_history[-1][0]
		self.vicon_pos.y=self.gazebo_location_history[-1][1]
		self.vicon_pos.z=self.gazebo_location_history[-1][2]
		del self.gazebo_location_history[-1]
		self.errors_calculation1()


	def ImuCb(self, msg):
		self.IMU_quat.x = msg.orientation.x
		self.IMU_quat.y = msg.orientation.y
		self.IMU_quat.z = msg.orientation.z
		self.IMU_quat.w = msg.orientation.w
		self.IMU_rot_euler=self.tF.get_eulerzxy_from_quaternion(self.IMU_quat)

	def wall_reaction_Cb(self,msg):
		self.wall_reaction=msg.data
		print(self.wall_reaction)

	def RCCb_simulation(self):
		csvreader=csv.reader(open('src/ol/setpoint_files/RC_simulated.csv'))
		for row in csvreader:
			self.RC_values=[int(row[0]),int(row[1]),int(row[2]),int(row[3]),int(row[4]),int(row[5]),int(row[6]),int(row[7])]
	def RCCb(self,msg):
		self.RC_values=msg.channels


	
	def errors_calculation1(self):
		self.pos_error[0][0]=self.position_target.x-self.vicon_pos.x
		self.pos_error[1][0]=self.position_target.y-self.vicon_pos.y
		self.pos_error[2][0]=self.position_target.z-self.vicon_pos.z
		#print(self.pos_error)
        
		dt=rospy.get_time()-self.prev_time
		dt_prev=self.prev_time-self.prev_prev_time
        
		if dt>=(0.8/self.vicon_frequency) and dt_prev>=(0.8/self.vicon_frequency):
			self.df[0][0]=(1-(dt/self.tau))*self.df_prev[0][0]+(self.vicon_pos_prev.x-self.vicon_pos.x)/self.tau
			self.df[1][0]=(1-(dt/self.tau))*self.df_prev[1][0]+(self.vicon_pos_prev.y-self.vicon_pos.y)/self.tau
			self.df[2][0]=(1-(dt/self.tau))*self.df_prev[2][0]+(self.vicon_pos_prev.z-self.vicon_pos.z)/self.tau


		if abs(self.pos_error[0][0])<self.integral_error_influence_dist and self.vicon_pos.z>self.floor_elevation:
			self.error_integral[0][0]=min(max(self.error_integral[0][0]+self.pos_error[0][0]*dt,-self.integral_error_limit),self.integral_error_limit)
		if abs(self.pos_error[1][0])<self.integral_error_influence_dist and self.vicon_pos.z>self.floor_elevation:
			self.error_integral[1][0]=min(max(self.error_integral[1][0]+self.pos_error[1][0]*dt,-self.integral_error_limit),self.integral_error_limit)
		if abs(self.pos_error[2][0])<self.integral_error_influence_dist and self.vicon_pos.z>self.floor_elevation:
			self.error_integral[2][0]=min(max(self.error_integral[2][0]+self.pos_error[2][0]*dt,-self.integral_error_limit),self.integral_error_limit)
        
        #print("vicon location x,y,z,w: {}, {}, {}, {}".format(self.vicon_rot.x,self.vicon_rot.y,self.vicon_rot.z,self.vicon_rot.w))
		#print("Logging started")
		data=[self.prev_time,self.position_target.x,self.position_target.y,self.position_target.z,self.vicon_pos.x,self.vicon_pos.y,self.vicon_pos.z,self.pos_error[0][0],self.pos_error[1][0],self.pos_error[2][0],self.df[0][0],self.df[1][0],self.df[2][0],self.error_integral[0][0],self.error_integral[1][0],self.error_integral[2][0],self.rdes_ddot[0][0],self.rdes_ddot[1][0],self.rdes_ddot[2][0],self.thrust_physical,self.att.thrust,self.phi_des*180/math.pi,self.theta_des*180/math.pi,self.yaw_des*180/math.pi,self.att.orientation.x,self.att.orientation.y,self.att.orientation.z,self.att.orientation.w,self.vicon_rot.x,self.vicon_rot.y,self.vicon_rot.z,self.vicon_rot.w,self.vicon_rot_euler[0][0]*180/math.pi,self.vicon_rot_euler[1][0]*180/math.pi,self.vicon_rot_euler[2][0]*180/math.pi,self.IMU_rot_euler[0][0]*180/math.pi,self.IMU_rot_euler[1][0]*180/math.pi,self.IMU_rot_euler[2][0]*180/math.pi,self.link_rot_euler[0][0]*180/math.pi,self.link_rot_euler[1][0]*180/math.pi,self.link_rot_euler[2][0]*180/math.pi,self.wall_reaction]
		with open(FILE_NAME, 'a') as csvfile:
			csvwriter = csv.writer(csvfile)
			csvwriter.writerow(data)
			csvfile.close
        
		#print("Logging done")


		self.df_prev[0][0]=self.df[0][0]
		self.df_prev[1][0]=self.df[1][0]
		self.df_prev[2][0]=self.df[2][0]

		self.vicon_pos_prev_prev.x=self.vicon_pos_prev.x
		self.vicon_pos_prev_prev.y=self.vicon_pos_prev.y
		self.vicon_pos_prev_prev.z=self.vicon_pos_prev.z

		self.vicon_pos_prev.x=self.vicon_pos.x
		self.vicon_pos_prev.y=self.vicon_pos.y
		self.vicon_pos_prev.z=self.vicon_pos.z

		self.prev_prev_time=self.prev_time
		self.prev_time=rospy.get_time()

		self.average_thrust=((self.average_thrust*self.average_thrust_count)+self.att.thrust)/(self.average_thrust_count+1)
		self.average_thrust_count+=1

	def emergency_land(self):
		self.att.orientation.x =0.0
		self.att.orientation.y =0.0
		self.att.orientation.z =0.0
		self.att.orientation.w =1.0
		self.att.thrust =0.2
	
	def linkangle_cb(self,msg):
		self.link_rot.x=msg.transform.rotation.x
		self.link_rot.y=msg.transform.rotation.y
		self.link_rot.z=msg.transform.rotation.z
		self.link_rot.w=msg.transform.rotation.w
		print(self.link_rot)
		self.link_rot_euler=self.tF.get_eulerzxy_from_quaternion(self.link_rot)



	def outerloop(self):  
		
		csvreader = csv.reader(open('src/thrust_test/Setpoint_files/Setpoint.csv'))    
		"""
		for row in csvreader:
			self.position_target.x=max(min(float(row[1]),self.x_max),-self.x_max)
			self.position_target.y=max(min(float(row[2]),self.y_max),-self.y_max)
			self.position_target.z=max(min(float(row[3]),self.z_max),-1) 
		
		"""
		for row in csvreader:
			thrust = float(row[0])
		print(thrust)
		self.att.orientation.x =0.0
		self.att.orientation.y =0.0
		self.att.orientation.z =0.0
		self.att.orientation.w =1.0
		self.att.thrust =thrust
		print(self.att)
		
		return self.att
	
	def outerloop1(self):
		try:
			csvreader = csv.reader(open('src/thrust_test/Setpoint_files/Setpoint.csv'))
			
			thrust_limit_value=50
			
			tilt_limit_value=10*math.pi/180
			
			for row in csvreader:
				#print('start0')
				
				self.yaw_des = float(row[0])*math.pi/180
				self.position_target.x=max(min(float(row[1]),self.x_max),-self.x_max)				
				self.position_target.y=max(min(float(row[2]),self.y_max),-self.y_max)
				self.position_target.z=max(min(float(row[3]),self.z_max),-1)
				"""
				
				self.phi_des = float(row[3])*math.pi/180
				self.theta_des = float(row[4])*math.pi/180
				self.yaw_des = float(row[5])*math.pi/180
				self.thrust_normalized = float(row[6])
				"""
				#print(self.position_target)

			
			
			
			#print("start1")
				
			self.rdes_ddot=np.matmul(self.kp,self.pos_error)+np.matmul(self.kv,self.df_prev)+np.matmul(self.ki,self.error_integral)
			
			
			
			if self.rdes_ddot[2]>3.0:
				self.rdes_ddot[2]=3.0
			if self.rdes_ddot[2]<-3.0:
				self.rdes_ddot[2]=-3.0
			
			self.thrust_physical=max(min(self.mass*(self.g*0.96+self.rdes_ddot[2][0]),thrust_limit_value),0.001)
			#print('start2')
			self.phi_des=max(min((1/self.g)*(self.rdes_ddot[0][0]*math.sin(self.vicon_rot_euler[2][0])-self.rdes_ddot[1][0]*math.cos(self.vicon_rot_euler[2][0])),tilt_limit_value),-tilt_limit_value)
			self.theta_des=max(min((1/self.g)*(self.rdes_ddot[0][0]*math.cos(self.vicon_rot_euler[2][0])+self.rdes_ddot[1][0]*math.sin(self.vicon_rot_euler[2][0])),tilt_limit_value),-tilt_limit_value)
			
			desired_orientation_quat=self.tF.get_quaternion_from_euler((self.yaw_des-self.yaw_offset*math.pi/180),-self.phi_des,-self.theta_des)
			

			## Testing the motors working 
			
			
			self.thrust_normalized=round(((self.thrust_physical+1.39198)/43.00745)/self.scale,4)
			
			print(self.thrust_normalized)
			if self.thrust_normalized >4:
				self.thrust_normalized=3.8
			
			if (self.vicon_pos.z>=2.5):   # change before experiment
				self.thrust_normalized=0.2

			if RUN_TYPE=="hardware":
				self.att.orientation.x=desired_orientation_quat[0]
				self.att.orientation.y=desired_orientation_quat[1]
				self.att.orientation.z=desired_orientation_quat[2]
				self.att.orientation.w=desired_orientation_quat[3]
				self.att.thrust= self.thrust_normalized
			
			elif RUN_TYPE=="simulation":
				self.tele_comand_history.insert(0,[desired_orientation_quat[0],desired_orientation_quat[1],desired_orientation_quat[2],desired_orientation_quat[3],self.thrust_normalized])
				self.att.orientation.x=self.tele_comand_history[-1][0]
				self.att.orientation.y=self.tele_comand_history[-1][1]
				self.att.orientation.z=self.tele_comand_history[-1][2]
				self.att.orientation.w=self.tele_comand_history[-1][3]
				self.att.thrust=1.5*self.tele_comand_history[-1][4]
				del self.tele_comand_history[-1]

		except:
			self.emergency_land()
			print("Emergency land mode")
				