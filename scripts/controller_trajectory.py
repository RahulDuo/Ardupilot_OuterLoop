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
		
		
		self.state = State()
		self.position_target=Point()
		self.scale=1
		self.vicon_pos=Point(0.0,0.0,10.0)     
		self.vicon_pos_prev=Point(0.0,0.0,10.0)
		self.vicon_pos_prev_prev=Point(0.0,0.0,10.0)
		self.vicon_vel=Point(0.0,0.0,0.0)
		self.vicon_vel_prev=Point(0.0,0.0,0.0)
		self.vicon_vel_prev_prev=Point(0.0,0.0,0.0)
		self.vicon_vel_prev_prev_prev=Point(0.0,0.0,0.0)
        
		self.vicon_rot=Quaternion(0.0,0.0,0.0,0.0) 
		self.vicon_euler_prev=np.array([[0.0],[0.0],[0.0]])
		self.pos_error=np.array([[0.0],[0.0],[0.0]])
		self.pos_error_prev=np.array([[0.0],[0.0],[0.0]])
		self.error_derivative=np.array([[0.0],[0.0],[0.0]])
		self.error_derivative_prev=np.array([[0.0],[0.0],[0.0]])
		self.error_derivative_prev_prev=np.array([[0.0],[0.0],[0.0]])
		self.error_derivative_prev_prev_prev=np.array([[0.0],[0.0],[0.0]])
		self.error_integral=np.array([[0.0],[0.0],[0.0]])

		self.rms=0.0
		self.rms_points=1
		self.mass = 1.6
		self.average_thrust=0.0
		self.average_thrust_count=0
		# Instantiate a Attitude target message
        
        #==================================================== Configuration Parameters =======================================
		self.thrust_physical=0
        
		self.att = AttitudeTarget()
		self.att.type_mask = int('00111', 2)
		self.att.orientation.x=0
		self.att.orientation.y=0
		self.att.orientation.z=0
		self.att.orientation.w=1
		self.att.thrust=0.01
        
		self.desired_vel=np.array([[0.0],[0.0],[0.0]])
		self.desired_acc=np.array([[0.0],[0.0],[0.0]])
    
		self.g=9.81 #m/s^2
		self.rdes_ddot=np.array([[0.0],[0,0],[0.0]])
		self.phi_des=0.0
		self.theta_des=0.0
		self.yaw_des=0.0
		self.prev_time=rospy.get_time()
		self.prev_prev_time=rospy.get_time()
		self.vicon_publishing=False

		#============================================= Configuration Parameters ======================================
    	
    	#== Trajectory Definition ======
		self.r=1.0 #m
		self.z_init=1.5#m
		self.z_final=1.5  	#m
		self.t_takeoff=15     #s
		self.t_positioning=6 #s
		self.with_yaw=True
		self.t_helix=17    #s
        #================================

        

		self.yaw_offset=0 #deg (value of vicon yaw when pixhawk shows 1.57 yaw angle)
		self.thrust_limit_value=50 #N
		self.tilt_limit_value=20 #Degrees
		self.integral_error_limit=6
		self.integral_error_influence_dist=0.5 #m
		self.integral_elevation=0.25
		self.floor_elevation=0.23 #m

		""""""
		""""""
		if RUN_TYPE=="simulation":
			self.vicon_frequency=250.0 #Hz
		elif RUN_TYPE=="hardware":
			self.vicon_frequency=125.0 #Hz

		self.control_loop_rate=250.0/3
		
		if RUN_TYPE=="hardware":
			self.kp=np.array([[2.2,0,0],[0,2.2,0],[0,0,3.6]])
			self.kv=np.array([[3.6,0,0],[0,3.6,0],[0,0,5]])
			self.ki=np.array([[1.0,0,0],[0,1.0,0],[0,0,0.5]])

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
		print(self.pos_error)
		dt=rospy.get_time()-self.prev_time
		dt_prev=self.prev_time-self.prev_prev_time

		if dt>=(1.0/(self.vicon_frequency*1.5)) and dt_prev>=(1.0/(self.vicon_frequency*1.5)):
			self.vicon_vel.x=0.93*self.vicon_vel_prev.x+0.035*(self.vicon_pos.x-self.vicon_pos_prev.x)/dt+0.035*(self.vicon_pos_prev.x-self.vicon_pos_prev_prev.x)/dt_prev
			self.vicon_vel.y=0.93*self.vicon_vel_prev.y+0.035*(self.vicon_pos.y-self.vicon_pos_prev.y)/dt+0.035*(self.vicon_pos_prev.y-self.vicon_pos_prev_prev.y)/dt_prev
			self.vicon_vel.z=0.5*self.vicon_vel_prev.z+0.25*(self.vicon_pos.z-self.vicon_pos_prev.z)/dt+0.25*(self.vicon_pos_prev.z-self.vicon_pos_prev_prev.z)/dt_prev

		if abs(self.pos_error[0][0])<self.integral_error_influence_dist and self.vicon_pos.z>self.integral_elevation:
			self.error_integral[0][0]=min(max(self.error_integral[0][0]+self.pos_error[0][0]*dt,-self.integral_error_limit),self.integral_error_limit)
		if abs(self.pos_error[1][0])<self.integral_error_influence_dist and self.vicon_pos.z>self.integral_elevation:
			self.error_integral[1][0]=min(max(self.error_integral[1][0]+self.pos_error[1][0]*dt,-self.integral_error_limit),self.integral_error_limit)
		if abs(self.pos_error[2][0])<self.integral_error_influence_dist and self.vicon_pos.z>self.integral_elevation:
			self.error_integral[2][0]=min(max(self.error_integral[2][0]+self.pos_error[2][0]*dt,-self.integral_error_limit),self.integral_error_limit)

		self.velocity_error=self.desired_vel-np.array([[self.vicon_vel.x],[self.vicon_vel.y],[self.vicon_vel.z]])
        
		self.vicon_vel_prev.x=self.vicon_vel.x
		self.vicon_vel_prev.y=self.vicon_vel.y
		self.vicon_vel_prev.z=self.vicon_vel.z

		self.vicon_pos_prev_prev.x=self.vicon_pos_prev.x
		self.vicon_pos_prev_prev.y=self.vicon_pos_prev.y
		self.vicon_pos_prev_prev.z=self.vicon_pos_prev.z

		self.vicon_pos_prev.x=self.vicon_pos.x
		self.vicon_pos_prev.y=self.vicon_pos.y
		self.vicon_pos_prev.z=self.vicon_pos.z

		self.average_thrust=((self.average_thrust*self.average_thrust_count)+self.att.thrust)/(self.average_thrust_count+1)
		self.average_thrust_count+=1

		self.prev_prev_time=self.prev_time
		self.prev_time=rospy.get_time()
        #============================================================ Data Logging ============================================
		data=[self.prev_time,self.position_target.x,self.position_target.y,self.position_target.z,self.vicon_pos.x,self.vicon_pos.y,self.vicon_pos.z,self.pos_error[0][0],self.pos_error[1][0],self.pos_error[2][0],self.desired_vel[0][0],self.desired_vel[1][0],self.desired_vel[2][0],self.vicon_vel.x,self.vicon_vel.y,self.vicon_vel.z,self.velocity_error[0][0],self.velocity_error[1][0],self.velocity_error[2][0],self.desired_acc[0][0],self.desired_acc[1][0],self.desired_acc[2][0],self.error_integral[0][0],self.error_integral[1][0],self.error_integral[2][0],self.rdes_ddot[0][0],self.rdes_ddot[1][0],self.rdes_ddot[2][0],self.thrust_physical,self.att.thrust,self.phi_des*180/math.pi,self.theta_des*180/math.pi,self.yaw_des*180/math.pi,self.att.orientation.x,self.att.orientation.y,self.att.orientation.z,self.att.orientation.w,self.vicon_rot.x,self.vicon_rot.y,self.vicon_rot.z,self.vicon_rot.w,self.vicon_rot_euler[0][0]*180/math.pi,self.vicon_rot_euler[1][0]*180/math.pi,self.vicon_rot_euler[2][0]*180/math.pi]
		with open(FILE_NAME, 'a') as csvfile:
			csvwriter = csv.writer(csvfile)
			csvwriter.writerow(data)
			csvfile.close
	
	def trajectory_matrix(self):
		self.M=np.array([[1,self.t0,self.t0**2,self.t0**3,self.t0**4,self.t0**5],
                         [0,1,2*self.t0,3*self.t0**2,4*self.t0**3,5*self.t0**4],
                         [0,0,2,6*self.t0,12*self.t0**2,20*self.t0**3],
                         [1,self.tf,self.tf**2,self.tf**3,self.tf**4,self.tf**5],
                         [0,1,2*self.tf,3*self.tf**2,4*self.tf**3,5*self.tf**4],
                         [0,0,2,6*self.tf,12*self.tf**2,20*self.tf**3]])
		
	def trajectory(self,t):
		if t<=self.t_takeoff:
			self.t0=0
			self.tf=self.t_takeoff
			self.trajectory_matrix()
			b=np.array([[0,0,self.floor_elevation],[0,0,0],[0,0,0],[0,0,self.z_init],[0,0,0],[0,0,0]])
			a=np.linalg.solve(self.M,b)
			out     = np.array([a[0] + a[1]*t + a[2]*t**2 + a[3]*t**3 + a[4]*t**4 + a[5]*t**5])
			outd    = np.array([a[1] + 2*a[2]*t + 3*a[3]*t**2 + 4*a[4]*t**3 + 5*a[5]*t**4])
			outdd   = np.array([2*a[2] + 6*a[3]*t + 12*a[4]*t**2 + 20*a[5]*t**3])
			pos=Point(out[0][0],out[0][1],out[0][2])
			vel=np.array([[outd[0][0]],[outd[0][1]],[outd[0][2]]])
			acc=np.array([[outdd[0][0]],[outdd[0][1]],[outdd[0][2]]])
			yaw=0
			yawdot=0

		elif t>self.t_takeoff and t<=self.t_takeoff+self.t_positioning:
			self.t0=self.t_takeoff
			self.tf=self.t_takeoff+self.t_positioning
			self.trajectory_matrix()
			b=np.array([[0,0,self.z_init],[0,0,0],[0,0,0],[self.r,0,self.z_init],[0,0,0],[0,0,0]])
			a=np.linalg.solve(self.M,b)
			out     = np.array([a[0] + a[1]*t + a[2]*t**2 + a[3]*t**3 + a[4]*t**4 + a[5]*t**5])
			outd    = np.array([a[1] + 2*a[2]*t + 3*a[3]*t**2 + 4*a[4]*t**3 + 5*a[5]*t**4])
			outdd   = np.array([2*a[2] + 6*a[3]*t + 12*a[4]*t**2 + 20*a[5]*t**3])
			pos=Point(out[0][0],out[0][1],out[0][2])
			vel=np.array([[outd[0][0]],[outd[0][1]],[outd[0][2]]])
			acc=np.array([[outdd[0][0]],[outdd[0][1]],[outdd[0][2]]])
			yaw=0
			yawdot=0


		elif t>self.t_takeoff+self.t_positioning and t<self.t_takeoff+self.t_positioning+self.t_helix:
			self.rms=((self.rms*self.rms_points)+abs(((self.vicon_pos.x**2+self.vicon_pos.y**2)**0.5)-0.7))/(self.rms_points+1)
			self.rms_points+=1
			self.t0=self.t_takeoff+self.t_positioning
			self.tf=self.t_takeoff+self.t_positioning+self.t_helix
			self.trajectory_matrix()
			b=np.array([[0,self.z_init],[0,0],[0,0],[2*np.pi,self.z_final],[0,0],[0,0]])
			a=np.linalg.solve(self.M,b)
			out     = np.array([a[0] + a[1]*t + a[2]*t**2 + a[3]*t**3 + a[4]*t**4 + a[5]*t**5])
			outd    = np.array([a[1] + 2*a[2]*t + 3*a[3]*t**2 + 4*a[4]*t**3 + 5*a[5]*t**4])
			outdd   = np.array([2*a[2] + 6*a[3]*t + 12*a[4]*t**2 + 20*a[5]*t**3])
			beta    = out[0][0]
			betad   = outd[0][0]
			betadd  = outdd[0][0]
			z       = out[0][1]
			zd      = outd[0][1]
			zdd     = outdd[0][1]
            # position
			x       = np.cos(beta)*self.r
			y       = np.sin(beta)*self.r
			pos     = Point(x,y,z)
            # velocity
			xd      = -y*betad
			yd      =  x*betad
			vel     = np.array([[xd],[yd],[zd]])
            # acceleration
			xdd     = -x*betad**2 - y*betadd
			ydd     = -y*betad**2 + x*betadd
			acc     = np.array([[xdd],[ydd],[zdd]])

			if self.with_yaw:
				yaw = math.atan2(yd,xd)-math.pi/2
			else:
				yaw=0
			yawdot = 0

		elif t>self.t_takeoff+self.t_positioning+self.t_helix and t<=self.t_takeoff+self.t_positioning+self.t_helix+self.t_positioning:
			self.t0=self.t_takeoff+self.t_positioning+self.t_helix
			self.tf=self.t_takeoff+self.t_positioning+self.t_helix+self.t_positioning
			self.trajectory_matrix()
			b=np.array([[self.r,0,self.z_final],[0,0,0],[0,0,0],[0,0,self.z_final],[0,0,0],[0,0,0]])
			a=np.linalg.solve(self.M,b)
			out     = np.array([a[0] + a[1]*t + a[2]*t**2 + a[3]*t**3 + a[4]*t**4 + a[5]*t**5])
			outd    = np.array([a[1] + 2*a[2]*t + 3*a[3]*t**2 + 4*a[4]*t**3 + 5*a[5]*t**4])
			outdd   = np.array([2*a[2] + 6*a[3]*t + 12*a[4]*t**2 + 20*a[5]*t**3])
			pos=Point(out[0][0],out[0][1],out[0][2])
			vel=np.array([[outd[0][0]],[outd[0][1]],[outd[0][2]]])
			acc=np.array([[outdd[0][0]],[outdd[0][1]],[outdd[0][2]]])
			yaw=0
			yawdot=0

		else:
			self.t0=self.t_takeoff+self.t_positioning+self.t_helix+self.t_positioning
			self.tf=self.t_takeoff+self.t_positioning+self.t_helix+self.t_positioning+self.t_takeoff
			self.trajectory_matrix()
			b=np.array([[0,0,self.z_final],[0,0,0],[0,0,0],[0,0,self.floor_elevation],[0,0,0],[0,0,0]])
			a=np.linalg.solve(self.M,b)
			out     = np.array([a[0] + a[1]*t + a[2]*t**2 + a[3]*t**3 + a[4]*t**4 + a[5]*t**5])
			outd    = np.array([a[1] + 2*a[2]*t + 3*a[3]*t**2 + 4*a[4]*t**3 + 5*a[5]*t**4])
			outdd   = np.array([2*a[2] + 6*a[3]*t + 12*a[4]*t**2 + 20*a[5]*t**3])
			pos=Point(out[0][0],out[0][1],out[0][2])
			vel=np.array([[outd[0][0]],[outd[0][1]],[outd[0][2]]])
			acc=np.array([[outdd[0][0]],[outdd[0][1]],[outdd[0][2]]])
			yaw=0
			yawdot=0
    	# output desired state
		self.position_target = pos
		self.desired_vel = vel
		self.desired_acc = acc
		self.yaw_des = yaw
		self.desired_yawdot = yawdot
    	#print(self.desired_acc)
    #=========================================================================================================================

#=========================================

	def emergency_land(self):
		self.att.orientation.x =0.0
		self.att.orientation.y =0.0
		self.att.orientation.z =0.0
		self.att.orientation.w =1.0
		self.att.thrust =0.3


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
				
						
			self.rdes_ddot=self.desired_acc+np.matmul(self.kv,self.velocity_error)+np.matmul(self.kp,self.pos_error)+np.matmul(self.ki,self.error_integral)
			#print("1")
			if self.rdes_ddot[2]>3.0:
				self.rdes_ddot[2]=3.0
			if self.rdes_ddot[2]<-3.0:
				self.rdes_ddot[2]=-3.0
			self.phi_des=max(min((1/self.g)*(self.rdes_ddot[0][0]*math.sin(self.yaw_des)-self.rdes_ddot[1][0]*math.cos(self.yaw_des))+0*math.pi/180,self.tilt_limit_value*math.pi/180),-self.tilt_limit_value*math.pi/180)
			#print("2")
			self.theta_des=max(min((1/self.g)*(self.rdes_ddot[0][0]*math.cos(self.yaw_des)+self.rdes_ddot[1][0]*math.sin(self.yaw_des))+0*math.pi/180,self.tilt_limit_value*math.pi/180),-self.tilt_limit_value*math.pi/180)
			#print("3")
			#rotation_matrix_3_3=2*(self.vicon_rot.w * self.vicon_rot.w + self.vicon_rot.z * self.vicon_rot.z)-1
			self.thrust_physical=max(min(self.mass*(self.g*0.9+self.rdes_ddot[2][0])/(math.cos(self.phi_des)*math.cos(self.theta_des)),self.thrust_limit_value),0)
        	#self.thrust_physical=max(min(self.mass*(self.g+self.rdes_ddot[2][0])/rotation_matrix_3_3,self.thrust_limit_value),0)
        	#self.thrust_physical=max(min(self.mass*(self.g+self.rdes_ddot[2][0]),self.thrust_limit_value),0)
			
			self.thrust_normalized=round(((self.thrust_physical+1.39198)/42.00745),4)
			if self.thrust_normalized >4:
				self.thrust_normalized=3.9
			print(self.thrust_normalized)
			
			desired_orientation_quat=self.tF.get_quaternion_from_euler(self.yaw_des-(self.yaw_offset*math.pi/180),-self.phi_des,-self.theta_des)
			
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
				