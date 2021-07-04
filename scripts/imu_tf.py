#!/usr/bin/env python

import rospy
import numpy as np
from math import sin, cos
from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

#############################################################################
class imu_tf:
#############################################################################
    
    #############################################################################
    def __init__(self):
    #############################################################################
        rospy.init_node("imu_tf")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)  #10260
        
        #### parameters #######
        self.rate = rospy.get_param('~rate',10.0)  # the rate at which to publish the transform
        self.imu = Imu()
        
        # subscribers
        rospy.Subscriber("imu/quat", Quaternion, self.quatCallback)
        rospy.Subscriber("mpu/accel", Vector3, self.accelCallback)
        rospy.Subscriber("mpu/rpy", Vector3, self.rpyCallback)
        rospy.Subscriber("imu/heading", Float32, self.headingCallback)
        
        # publishers
        self.imuPub = rospy.Publisher("imu/data", Imu, queue_size = 10)
        self.imuEarthPub = rospy.Publisher("imu/earth", Imu, queue_size = 10)
        
    #############################################################################
    def spin(self):
    #############################################################################
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
            
    #############################################################################
    def update(self):
    #############################################################################
        self.imu.header.frame_id = "imu_link"
        self.imu.header.stamp = rospy.Time.now()
        self.imuPub.publish(self.imu)
            
    #############################################################################
    def quatCallback(self, msg):
    #############################################################################
        qArray = [msg.x, msg.y, msg.z, msg.w]/np.linalg.norm([msg.x, msg.y, msg.z, msg.w])
        self.imu.orientation.x = qArray[0]
        self.imu.orientation.y = -qArray[1]
        self.imu.orientation.z = -qArray[2]
        self.imu.orientation.w = qArray[3]
        
    #############################################################################
    def accelCallback(self, msg):
    #############################################################################
        self.imu.linear_acceleration.x = msg.x
        self.imu.linear_acceleration.y = -msg.x
        self.imu.linear_acceleration.z = -msg.z
        
    #############################################################################
    def rpyCallback(self, msg):
    #############################################################################
        self.imu.angular_velocity.x = msg.x
        self.imu.angular_velocity.y = -msg.y
        self.imu.angular_velocity.z = -msg.z
        
    #############################################################################
    def headingCallback(self, msg):
    #############################################################################
        imu_earth = Imu()
        imu_earth.header.frame_id = "odom"
        imu_earth.header.stamp = rospy.Time.now()
        imu_earth.orientation.x = 0
        imu_earth.orientation.y = 0
        imu_earth.orientation.z = sin(msg.data / 2)
        imu_earth.orientation.w = cos(msg.data / 2)
        self.imuEarthPub.publish(imu_earth)
        
#############################################################################
#############################################################################        
if __name__ == "__main__":
    imu = imu_tf()
    imu.spin()
        
