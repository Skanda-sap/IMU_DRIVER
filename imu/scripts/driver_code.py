#!/usr/bin/env python3
#package imports
import serial
import rospy
import datetime
import numpy as np

#sensor message import

from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

#serial port and baud rate setup

ser = serial.Serial('/dev/ttyUSB0')
ser.baudrate = 115200
#assigning variables to inbuilt functions

msg = Imu()
msg_mag = MagneticField()

#function for quaternion conversion

def quaternions(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

#driver function	

def imu_driv():
    pub_imu = rospy.Publisher('imu_publisher', Imu, queue_size=10)
    pub_mag = rospy.Publisher('mag_publisher', MagneticField, queue_size=10)
    rospy.init_node('imu_data', anonymous=True)
    r = rospy.Rate(40)
       
    while not rospy.is_shutdown():
        
        #reading serial inputs  
        inputs = str(ser.readline())
        print(inputs)
        
        #splitting with b' value from i/p
        string_splits=inputs.split("b'")
        
        #comma splitter
        strings=string_splits[1].split(",")
        print(strings)
        
        #accessing input values

        yaw=float(strings[1])
        pitch=float(strings[2])
        roll=float(strings[3])
        magx=float(strings[4])
        magy=float(strings[5])
        magz=float(strings[6])
        accelx=float(strings[7])
        accely=float(strings[8])
        accelz=float(strings[9])
        gyrox=float(strings[10])
        gyroy=float(strings[11])
        gyroz=strings[12]
        gyroz1=float(gyroz[:10])

        #calling quaternion function
        oreintations=quaternions(roll,pitch,yaw)
    
        msg.orientation.x=oreintations[0]
        msg.orientation.y=oreintations[1]
        msg.orientation.z=oreintations[2]
        msg.orientation.w=oreintations[3]

        msg.linear_acceleration.x=accelx
        msg.linear_acceleration.y=accely
        msg.linear_acceleration.z=accelz

        msg_mag.magnetic_field.x=magx
        msg_mag.magnetic_field.y=magy
        msg_mag.magnetic_field.z=magz

        msg.angular_velocity.x=gyrox
        msg.angular_velocity.y=gyroy
        msg.angular_velocity.z=gyroz1

	#publishing messages

        rospy.loginfo(msg)
        pub_imu.publish(msg)

        rospy.loginfo(msg_mag)
        pub_mag.publish(msg_mag)

        r.sleep()
if __name__ == '__main__':
    imu_driv()

