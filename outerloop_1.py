#!/usr/bin/env python
# ROS python API
import rospy

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from geometry_msgs.msg import TransformStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import BatteryState
import csv
import numpy as np
import math
from scipy.spatial.transform import Rotation as R

# Flight modes class
# Flight modes are activated using ROS services

########################## Attitude Bitmask Information #######################################
# Message for SET_ATTITUDE_TARGET
#
# Some complex system requires all feautures that mavlink
# message provide. See issue #402, #418.

#std_msgs/Header header

#uint8 type_mask
#uint8 IGNORE_ROLL_RATE  = 1 # body_rate.x
#uint8 IGNORE_PITCH_RATE = 2 # body_rate.y
#uint8 IGNORE_YAW_RATE   = 4 # body_rate.z
#uint8 IGNORE_THRUST     = 64
#uint8 IGNORE_ATTITUDE   = 128 # orientation field

#geometry_msgs/Quaternion orientation
#geometry_msgs/Vector3 body_rate
#float32 thrust
########################### Position Bitmask Information #######################################

################################################################################################
# Message for SET_POSITION_TARGET_LOCAL_NED
#
# Some complex system requires all feautures that mavlink
# message provide. See issue #402.

#std_msgs/Header header

#uint8 coordinate_frame
#uint8 FRAME_LOCAL_NED        = 1
#uint8 FRAME_LOCAL_OFFSET_NED = 7
#uint8 FRAME_BODY_NED         = 8
#uint8 FRAME_BODY_OFFSET_NED  = 9

#uint16 type_mask
#uint16 IGNORE_PX       = 1 # Position ignore flags
#uint16 IGNORE_PY       = 2
#uint16 IGNORE_PZ       = 4
#uint16 IGNORE_VX       = 8 # Velocity vector ignore flags
#uint16 IGNORE_VY       = 16
#uint16 IGNORE_VZ       = 32
#uint16 IGNORE_AFX      = 64 # Acceleration/Force vector ignore flags
#uint16 IGNORE_AFY      = 128
#uint16 IGNORE_AFZ      = 256
#uint16 FORCE           = 512 # Force in af vector flag
#uint16 IGNORE_YAW      = 1024
#uint16 IGNORE_YAW_RATE = 2048

#geometry_msgs/Point position
#geometry_msgs/Vector3 velocity
#geometry_msgs/Vector3 acceleration_or_force
#float32 yaw
#float32 yaw_rate
####################################################################################################



z_input=[0]

def get_quaternion_from_euler(psi,phi,th):
    rot_matrix_1=np.array([[np.cos(psi),-np.sin(psi),0],[np.sin(psi),np.cos(psi),0],[0,0,1]])
    rot_matrix_2=np.array([[1,0,0],[0,np.cos(phi),-np.sin(phi)],[0,np.sin(phi),np.cos(phi)]])
    rot_matrix_3=np.array([[np.cos(th),0,np.sin(th)],[0,1,0],[-np.sin(th),0,np.cos(th)]])
    rot_matrix=np.matmul(np.matmul(rot_matrix_1,rot_matrix_2),rot_matrix_3)
    qw=((1+rot_matrix[0][0]+rot_matrix[1][1]+rot_matrix[2][2])**0.5)/2
    qx=(rot_matrix[2][1]-rot_matrix[1][2])/(4*qw)
    qy=(rot_matrix[0][2]-rot_matrix[2][0])/(4*qw)
    qz=(rot_matrix[1][0]-rot_matrix[0][1])/(4*qw)
    return [qx, qy, qz, qw]
    
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

def get_rotmat_from_quaternion(quaternions):
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
    
class fcuModes:
    def __init__(self):
        pass

    def setTakeoff(self):
        rospy.wait_for_service('mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            takeoffService(altitude = 1)
        except rospy.ServiceException as e:
            print ("Service takeoff call failed: %s"%e)

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
            print("Arming")
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        print("Disarming")
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
            print("Disarming")
        except rospy.ServiceException as e:
            print ("Service disarming call failed: %s"%e)

    def setStabilizedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Stabilized Mode could not be set."%e)

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
            print("Offboard Mode Started")
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Offboard Mode could not be set."%e)

    def setAltitudeMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Altitude Mode could not be set."%e)

    def setPositionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
            print("Position mode")
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Position Mode could not be set."%e)

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException as e:
               print ("service set_mode call failed: %s. Autoland Mode could not be set."%e)


class Controller:
    # initialization method 
    def __init__(self):
        self.state = State()
        self.position_target=Point()
        self.vicon_pos=Point(0.0,0.0,0.0)
        self.vicon_pos_prev=Point(0.0,0.0,0.0)
        self.vicon_pos_prev_prev=Point(0.0,0.0,10.0)        
        self.vicon_rot=Quaternion(0.0,0.0,0.0,1.0)
        self.quats=Quaternion(0.0,0.0,0.0,1.0) 
        self.prev_time_controller=0.0
        self.pos_error=np.array([[0.0],[0.0],[0.0]])
        self.pos_error_prev=np.array([[0.0],[0.0],[0.0]])
        self.error_integral=np.array([[0.0],[0.0],[0.0]])

        self.RC_values=[0.0]*8
        
        
        self.df=np.array([[0.0],[0.0],[0.0]])                                       #the new filter 
        self.df_prev=np.array([[0.0],[0.0],[0.0]])
        self.tau=0.0333 #50Hz low-pass filter
        
        self.pos_error_prev_prev=np.array([[0.0],[0.0],[0.0]])

        self.UAV_rot_mat=np.array([[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]])
      
        
        #self.px4_attitude=Quaternion(0.0,0.0,0.0,0.0)
        # Instantiate a Attitude target message
        self.att = AttitudeTarget()
        self.att.type_mask = int('00111', 2)
        self.att.orientation.x=0
        self.att.orientation.y=0
        self.att.orientation.z=0
        self.att.orientation.w=1
        self.att.thrust=0.1
        self.thrust_physical=0.0
        self.scale=1
        self.rate_landing=0.5

        self.quaternion_s=1
        self.quaternion_v=np.array([[0.0],[0.0],[0.0]])
        
        self.g=9.81 #m/s^2
        self.ge3=np.array([[0.0],[0.0],[self.g]])
        self.rdes_ddot=np.array([[0.0],[0.0],[0.0]])
        self.phi_des=0.0
        self.yaw_des=0 #math.pi/2
        self.theta_des=0.0
        self.b3=np.array([0,0,1])
        self.b1=np.array([math.cos(self.yaw_des),math.sin(self.yaw_des),0])
        self.b2=np.cross(self.b3,self.b1)
        self.rot_matrix_des=np.array([[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]])
        self.prev_time=rospy.get_time()
        self.prev_prev_time=rospy.get_time()

        self.vicon_publishing=False
        self.average_thrust=0.0
        self.average_thrust_count=0

        self.battery_voltage_total=0.0
        self.battery_voltage_cells=[0.0,0.0,0.0]
        self.battery_current=0.0
        
        #=================================== Configuration Parameters ================================
        
        self.x_max=1 #m
        self.y_max=1 #m
        self.z_max=1.3 #m
        self.flag_wall=False
        #self.yaw_offset=-9.957 #deg (value of vicon yaw when pixhawk shows 1.57 yaw angle)
        self.yaw_offset=0 #deg
        #self.yaw_offset=0 #deg
        
        self.integral_error_limit=6
        self.integral_error_influence_dist=1 #m
        self.floor_elevation=0.25

        self.run_type="hardware"   #either "simulation" or "hardware"                             Not setting correctly this is dangerous, crash possible
        ################################################################
        self.control_loop_rate=250/3

        self.motor_type="emax_935kv_emax_esc"    #either "emax_935kv" or "2212" or "emax_935kv_emax_esc"

        if self.run_type=="simulation":
            self.vicon_frequency=250.0 #Hz
        elif self.run_type=="hardware":
            self.vicon_frequency=120.0 #Hz

        ###############################################################

        if self.run_type=="hardware":
            self.kp=np.array([[2,0,0],[0,2,0],[0,0,6]])
            self.kv=np.array([[3,0,0],[0,3,0],[0,0,5]])
            self.ki=np.array([[0.5,0,0],[0,0.5,0],[0,0,0.5]])
        elif self.run_type=="simulation":
            #pass
            self.kp=np.array([[3,0,0],[0,3,0],[0,0,6]])
            self.kv=np.array([[2.25,0,0],[0,2.25,0],[0,0,5]])
            self.ki=np.array([[0.3,0,0],[0,0.3,0],[0,0,0.35]])
        
        if self.run_type=="simulation":
            self.filename="src/ol/log_files/setpoint_tracking_simulation/2023_07_01_01.csv"       #for simulation
        elif self.run_type=="hardware":
            self.filename="src/ol/log_files/setpoint_tracking/2023_07_18_01.csv"    #for hardware experiment
        
        self.mass=1.2 #kg
        
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
        
        #=================================== Configuration Parameters Ends ==============================

    def RCCb(self,msg):
        self.RC_values=msg.channels[0:8]
        #print(msg)

    def RCCb_simulation(self):
        csvreader=csv.reader(open('src/ol/setpoint_files/RC_simulated.csv'))
        for row in csvreader:
            self.RC_values=[int(row[0]),int(row[1]),int(row[2]),int(row[3]),int(row[4]),int(row[5]),int(row[6]),int(row[7])]


    def viconCb(self,msg):
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
        self.vicon_rot_euler=get_eulerzxy_from_quaternion(self.vicon_rot)
        self.UAV_rot_mat=get_rotmat_from_quaternion(self.vicon_rot)
        # print("UAV Roll Pitch Yaw: {} {} {}".format(round(self.vicon_rot_euler[0][0]*180/math.pi,2),round(self.vicon_rot_euler[1][0]*180/math.pi,2),round(self.vicon_rot_euler[2][0]*180/math.pi,2)))
        self.errors_calculation()

    def GazeboCb(self,msg):
        self.vicon_publishing=True
        self.vicon_rot.x=msg.pose[1].orientation.x
        self.vicon_rot.y=msg.pose[1].orientation.y
        self.vicon_rot.z=msg.pose[1].orientation.z
        self.vicon_rot.w=msg.pose[1].orientation.w
        # sprint(self.vicon_rot)
        #print("vicon location x,y,z: {}, {}, {}".format(self.vicon_pos.x,self.vicon_pos.y,self.vicon_pos.z))
        self.vicon_rot_euler=get_eulerzxy_from_quaternion(self.vicon_rot)
        self.UAV_rot_mat=get_rotmat_from_quaternion(self.vicon_rot)
        
        # self.gazebo_location_history.insert(0,[msg.pose[1].position.x+self.gazebo_vicon_offset*math.sin(self.vicon_rot_euler[1][0]+np.random.normal(0,self.gazebo_std_dev_angular)*math.pi/180)+np.random.normal(0,self.gazebo_std_dev_xy/1000),msg.pose[1].position.y-self.gazebo_vicon_offset*math.sin(self.vicon_rot_euler[0][0]+np.random.normal(0,self.gazebo_std_dev_angular)*math.pi/180)+np.random.normal(0,self.gazebo_std_dev_xy/1000),msg.pose[1].position.z+self.gazebo_vicon_offset+np.random.normal(0,self.gazebo_std_dev_z/1000)])
        self.gazebo_location_history.insert(0,[msg.pose[1].position.x,msg.pose[1].position.y,msg.pose[1].position.z])
        self.vicon_pos.x=self.gazebo_location_history[-1][0]
        self.vicon_pos.y=self.gazebo_location_history[-1][1]
        self.vicon_pos.z=self.gazebo_location_history[-1][2]
        del self.gazebo_location_history[-1]
        self.errors_calculation()

    def errors_calculation(self):
        self.pos_error[0][0]=self.position_target.x-self.vicon_pos.x
        self.pos_error[1][0]=self.position_target.y-self.vicon_pos.y
        self.pos_error[2][0]=self.position_target.z-self.vicon_pos.z
        
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
        data=[self.prev_time,self.position_target.x,self.position_target.y,self.position_target.z,self.vicon_pos.x,self.vicon_pos.y,self.vicon_pos.z,self.pos_error[0][0],self.pos_error[1][0],self.pos_error[2][0],self.df[0][0],self.df[1][0],self.df[2][0],self.error_integral[0][0],self.error_integral[1][0],self.error_integral[2][0],self.rdes_ddot[0][0],self.rdes_ddot[1][0],self.rdes_ddot[2][0],self.thrust_physical,self.att.thrust,self.phi_des*180/math.pi,self.theta_des*180/math.pi,self.yaw_des*180/math.pi,self.att.orientation.x,self.att.orientation.y,self.att.orientation.z,self.att.orientation.w,self.vicon_rot.x,self.vicon_rot.y,self.vicon_rot.z,self.vicon_rot.w,self.vicon_rot_euler[0][0]*180/math.pi,self.vicon_rot_euler[1][0]*180/math.pi,self.vicon_rot_euler[2][0]*180/math.pi,self.kp[2][2]*self.pos_error[2][0],self.kv[2][2]*self.df[2][0]]
        with open(self.filename, 'a') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(data)
            csvfile.close
        
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

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg
        #print(msg)

    def batteryCb(self,msg):
        self.battery_voltage_total=round(msg.voltage,3)
        if self.run_type=="hardware":
            self.battery_voltage_cells=msg.cell_voltage
        self.battery_current=round(msg.current,3)

    def outerloop(self):
        try:
            csvreader=csv.reader(open('src/ol/setpoint_files/setpoint.csv'))
            thrust_limit_value=16
            tilt_limit_value=8*math.pi/180
            for row in csvreader:
                self.position_target.x=max(min(float(row[0]),self.x_max),-self.x_max)
                self.position_target.y=max(min(float(row[1]),self.y_max),-self.y_max)
                self.position_target.z=max(min(float(row[2]),self.z_max),-1)
                # self.yaw_des=np.deg2rad(float(row[3]))
            
            ######### Landing ##########
            if self.position_target.z==0.0:
                self.scale+=self.rate_landing
                print('landing')
            ############################
            ###### Virtual Wall Safety############
            if abs(self.vicon_pos.x)<=0.2 and abs(self.position_target.x)<=0.8 and abs(self.vicon_pos.y)<=0.2 and abs(self.position_target.y)<=1:
                self.flag_wall=False
            if (self.vicon_pos.x>=1.1 or self.vicon_pos.y>=1.7 or self.vicon_pos.z>=1.4) or self.flag_wall==True:
                self.position_target.x=0
                self.position_target.y=0
                self.position_target.z=0.4
                self.flag_wall=True
            #######################################
            self.rdes_ddot=np.matmul(self.kp,self.pos_error)+np.matmul(self.kv,self.df_prev)+np.matmul(self.ki,self.error_integral)
            # self.rdes_ddot=np.matmul(self.kp,np.tanh(self.pos_error))+np.matmul(self.kv,np.tanh(self.df_prev))+np.matmul(self.ki,self.error_integral)

            ################################ Euler-Angle method ######################################################
            self.thrust_physical=max(min(self.mass*(np.linalg.norm(self.ge3+self.rdes_ddot)),thrust_limit_value),0.001)

            self.phi_des=max(min((1/self.g)*(self.rdes_ddot[0][0]*math.sin(self.vicon_rot_euler[2][0])-self.rdes_ddot[1][0]*math.cos(self.vicon_rot_euler[2][0])),tilt_limit_value),-tilt_limit_value)
            self.theta_des=max(min((1/self.g)*(self.rdes_ddot[0][0]*math.cos(self.vicon_rot_euler[2][0])+self.rdes_ddot[1][0]*math.sin(self.vicon_rot_euler[2][0])),tilt_limit_value),-tilt_limit_value)

            desired_orientation_quat=get_quaternion_from_euler((self.yaw_des-self.yaw_offset*math.pi/180),self.phi_des,self.theta_des)

            ##########################################################################################################


            ############################### Quaternion Method ########################################################
            # self.thrust_physical=max(min(self.mass*(np.linalg.norm(self.ge3+self.rdes_ddot)),thrust_limit_value),0.001)

            # self.quaternion_s=np.sqrt(0.5*(1+(self.mass*(self.g+self.rdes_ddot[2][0])/self.thrust_physical)))
            # # print("s",self.quaternion_s)
            # self.quaternion_v=-0.5*(self.mass/(self.thrust_physical*self.quaternion_s))*np.array([[self.rdes_ddot[1][0]],[-1*self.rdes_ddot[0][0]],[0.0]])
            # # print("v",self.quaternion_v)
            
            # desired_orientation_quat=[self.quaternion_v[0][0], self.quaternion_v[1][0], self.quaternion_v[2][0], self.quaternion_s]
            ##########################################################################################################

            ############################### Rotation Matrix Method #############################################################
            # # self.rdes_ddot=np.matmul(self.kp,np.tanh(self.pos_error))+np.matmul(self.kv,np.tanh(self.df_prev))+np.matmul(self.ki,self.error_integral)

            # self.thrust_physical=max(min(self.mass*(np.linalg.norm(self.ge3+self.rdes_ddot)),thrust_limit_value),0.001)

            # ##################### Attitude Extraction ########################
            # b3=np.array((self.ge3+self.rdes_ddot)/(np.linalg.norm(self.ge3+self.rdes_ddot)))
            # self.b3=np.array([b3[0][0],b3[1][0],b3[2][0]])
            # self.b1=np.array([math.cos(self.yaw_des),math.sin(self.yaw_des),0])
            # self.b2=np.cross(self.b3,self.b1)
            # b2b3=np.cross(self.b2,self.b3)

            # self.rot_matrix_des=np.array([[b2b3[0],self.b2[0],self.b3[0]],[b2b3[1],self.b2[1],self.b3[1]],[b2b3[2],self.b2[2],self.b3[2]]])

            # rot = R.from_matrix(self.rot_matrix_des)
            # desired_orientation_quat=rot.as_quat()
            ####################################################################################################################
            
            self.thrust_normalized=round(((self.thrust_physical+1.39198)/42.00745)/self.scale,4)

            if (self.vicon_pos.z>=1.4):
                self.thrust_normalized=0.29

            if self.run_type=="hardware":
                self.att.orientation.x=desired_orientation_quat[0]
                self.att.orientation.y=desired_orientation_quat[1]
                self.att.orientation.z=desired_orientation_quat[2]
                self.att.orientation.w=desired_orientation_quat[3]
                self.att.thrust= self.thrust_normalized
            elif self.run_type=="simulation":
                self.tele_comand_history.insert(0,[desired_orientation_quat[0],desired_orientation_quat[1],desired_orientation_quat[2],desired_orientation_quat[3],self.thrust_normalized])
                self.att.orientation.x=self.tele_comand_history[-1][0]
                self.att.orientation.y=self.tele_comand_history[-1][1]
                self.att.orientation.z=self.tele_comand_history[-1][2]
                self.att.orientation.w=self.tele_comand_history[-1][3]
                self.att.thrust=self.tele_comand_history[-1][4]
                del self.tele_comand_history[-1]
            #print(self.RC_values[5])
            if self.RC_values[5]>1300:
                self.emergency_land()
            #print(self.att)
        except:
            self.emergency_land()
            #print(self.att)


    def emergency_land(self):
        quat=get_quaternion_from_euler(self.yaw_des-self.yaw_offset*math.pi/180,0,0)
        self.att.orientation.x=quat[0]
        self.att.orientation.y=quat[1]
        self.att.orientation.z=quat[2]
        self.att.orientation.w=quat[3]
        self.att.thrust=0.25

def main():
    rospy.init_node('setpoint_node',disable_signals=True, anonymous=False)                         # initiate node
    modes = fcuModes()                                                                            # flight mode object
    cnt = Controller()                                                                            # controller object
    rate = rospy.Rate(cnt.control_loop_rate)                                                                          # ROS loop rate
    pos_mode_wait=rospy.Rate(0.5)
    rospy.Subscriber('mavros/state', State, cnt.stateCb)                                          # Subscribe to drone state
    # rospy.Subscriber('/mavros/battery',BatteryState,cnt.batteryCb)
    att_pub=rospy.Publisher('mavros/setpoint_raw/attitude',AttitudeTarget,queue_size=1)           # Setpoint Attitude pubisher
    data=['Time','Target_x','Target_y','Target_z','current_x','current_y','current_z','error_x','error_y','error_z','df_x','df_y','df_z','error_int_x','error_int_y','error_int_z','rddot_x','rddot_y','rddot_z','Thrust_physical','thrust_command','phi_des','theta_des','yaw_des','quat_x_command','quat_y_command','quat_z_command','quat_w_command','quat_x_actual','quat_y_actual','quat_z_actual','quat_w_actual','phi_actual','theta_actual','psi_actual']
    with open(cnt.filename, 'w') as csvfile:                        
                csvwriter = csv.writer(csvfile)
                csvwriter.writerow(data)
                csvfile.close                
    if cnt.run_type=="hardware":
        rospy.Subscriber('/vicon/Rahul_Drone_1/Rahul_Drone_1',TransformStamped, cnt.viconCb)              # vicon pose subscriber for hardware experiment
        # rospy.Subscriber('/mavros/rc/in',RCIn, cnt.RCCb)
    elif cnt.run_type=="simulation":
        rospy.Subscriber('/gazebo/model_states',ModelStates, cnt.GazeboCb)                            # Gazebo pose subscriber for simulation
    rate.sleep()
    t_wait=rospy.get_time()

    while not cnt.vicon_publishing==True:
        print("Waiting for vicon signal")
        if rospy.get_time()-t_wait>10.0:
            print("\n\n\n####################################\n   vicon DATA NOT RECEIVED in 10s\n####################################\n\n\n")
            raise KeyboardInterrupt
            break
        rate.sleep()

    
    # while cnt.battery_voltage_total==0.0:
    #     rate.sleep()

    while not cnt.state.armed:
        modes.setArm()
        rate.sleep()
    
    modes.setPositionMode()
    
    pos_mode_wait.sleep()
    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
    k=0
    while k<10:
        att_pub.publish(cnt.att)
        rate.sleep()
        k = k + 1

    # activate OFFBOARD mode
    modes.setOffboardMode()
    # ROS main loop
    cnt.prev_time=rospy.get_time()
    try:
        while rospy.get_time()-cnt.prev_time<2.0:
            cnt.outerloop()
            att_pub.publish(cnt.att)
            #print(rospy.get_time()-cnt.prev_time)
            #print(cnt.att)
            rate.sleep()
        print("\n\n\n#############################\n   vicon DATA TIMEOUT \n#############################\n\n\n")
        while True:
            cnt.emergency_land()
            att_pub.publish(cnt.att)
            #print(cnt.att)
            rate.sleep()
            
    except KeyboardInterrupt:
        while True:
            cnt.emergency_land()
            att_pub.publish(cnt.att)
            #print(cnt.att)
            rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass