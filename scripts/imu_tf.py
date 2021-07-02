#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import Imu, MagneticField

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
        self.mag = MagneticField()
        # subscribers
        rospy.Subscriber("mpu/quat", Quaternion, self.quatCallback)
        rospy.Subscriber("mpu/accel", Vector3, self.accelCallback)
        rospy.Subscriber("mpu/rpy", Vector3, self.rpyCallback)
        rospy.Subscriber("mag", Vector3, self.magCallback)
        
        # publishers
        self.imuPub = rospy.Publisher("imu/data_raw", Imu, queue_size = 10)
        self.magPub = rospy.Publisher("imu/mag", MagneticField, queue_size = 10)
        
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
        self.mag.header.frame_id = "mag_link"
        self.mag.header.stamp = rospy.Time.now()
        self.magPub.publish(self.mag)
            
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
        self.imu.linear_acceleration = msg
        
    #############################################################################
    def rpyCallback(self, msg):
    #############################################################################
        self.imu.angular_velocity = msg
        
    #############################################################################
    def magCallback(self, msg):
    #############################################################################
        self.mag.magnetic_field = msg    

#############################################################################
#############################################################################        
if __name__ == "__main__":
    imu = imu_tf()
    imu.spin()
        
