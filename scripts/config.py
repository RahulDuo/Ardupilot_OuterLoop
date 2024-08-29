from geometry_msgs.msg import Point,Pose, PoseStamped, Twist, Quaternion
from geometry_msgs.msg import TransformStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
import numpy as np
import math

MASS_UAV=1.7
g=9.81
THRUST_LIMIT=45
TILT_LIMIT_DEG=10
INTEGRAL_ERROR_LIMIT=10
INTEGRAL_ERROR_INFLUENCE_DIST=1
FLOOR_ELEVATION=0.3
# what are these values doing
TRANS_POS_ERROR_INTEGRAL_LIMIT=Point(10.0,10.0,10.0)
TRANS_POS_MAX=Point(1.5,1.5,2.3)

YAW_DES_DEG=0

RUN_TYPE="hardware"

if RUN_TYPE=="simulation":
	FILE_NAME="src/am_simulation/log_data/2024_02_12_01.csv"       #for simulation
	KP=np.array([1.2*2,1.2*2,1.2*5])
	KD=np.array([1.2*3,1.2*3,1.2*5])
	KI=np.array([1.0*0.5,1.0*0.5,5.0*0.5])
	KP=np.array([3.0*2,3.0*2,3.0*6])
	KD=np.array([2.0*3,2.0*3,2.0*5])
	KI=np.array([1.0*0.5,1.0*0.5,5.0*0.5])
	
elif RUN_TYPE=="hardware":
	FILE_NAME="src/am_simulation/log_data/2024_06_23_03.csv"    #for hardware experiment
	"""
	
	kp=np.array([1.2*2,1.0*2,0.6*6])
	kd=np.array([1.2*3,1.2*3,0.75*5])
	ki=np.array([2.0*0.5,2.0*0.5,5.0*0.5])
	
	kp=np.array([[2.4,0,0],[0,2.0,0],[0,0,3.6]])
	kv=np.array([[3.6,0,0],[0,3.6,0],[0,0,3.85]])
	ki=np.array([[1,0,0],[0,1,0],[0,0,2.5]])
	"""