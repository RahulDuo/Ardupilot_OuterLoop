#!/usr/bin/env python
import time
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
import os
import rospy
import sys
import serial
rospy.init_node('encoder_node')
rate = rospy.Rate(10)
ser = serial.Serial(port='/dev/ttyUSB1',baudrate = 115200,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS,timeout=0.003)
SP_pub=rospy.Publisher('/wall_reaction',Float64,queue_size=1)
def wall_reaction_Cb(msg):
    wall_reaction=msg.data
    print(wall_reaction)
rospy.Subscriber("/wall_reaction",Float64,wall_reaction_Cb)
print(ser)
while True:
    while ser.in_waiting:
        Encoder_read=ser.readline(None).split()
        print(Encoder_read)
        #print("heelo")
        try:
            SP_pub.publish(float(Encoder_read[-1]))
            Encoder_read_prev=float(Encoder_read[-1])
        except ValueError:
            SP_pub.publish(Encoder_read_prev)