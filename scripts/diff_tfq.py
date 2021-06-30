#!/usr/bin/env python

# Converts encoder ticks from wheel to odometry
# Uses quaternions from MPU6050 for orientationr

import rospy
from math import sin, cos, asin
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16
from geometry_msgs.msg import Quaternion
from tf.broadcaster import TransformBroadcaster
import numpy as np

#############################################################################
class DiffTfq:
#############################################################################

    #############################################################################
    def __init__(self):
    #############################################################################
        rospy.init_node("diff_tfq")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)  #10260
        
        #### parameters #######
        self.rate = rospy.get_param('~rate',10.0)  # the rate at which to publish the transform
        self.ticks_meter = float(rospy.get_param('ticks_meter', 96))  # The number of wheel encoder ticks per meter of travel
        self.base_width = float(rospy.get_param('~base_width', 0.24)) # The wheel base width in meters
        self.control = rospy.get_param('~control_bit',1)
        self.base_frame_id = rospy.get_param('~base_frame_id','base_footprint') # the name of the base frame of the robot
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom') # the name of the odometry reference frame
        
        self.encoder_min = rospy.get_param('encoder_min', -32678)
        self.encoder_max = rospy.get_param('encoder_max', 32677)
        self.encoder_low_wrap = rospy.get_param('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min )
        self.encoder_high_wrap = rospy.get_param('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min )
 
        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta
        
        # internal data
        self.enc_left = None        # wheel encoder readings
        self.enc_right = None
        self.left = 0               # actual values coming back from robot
        self.right = 0
        self.lmult = 0
        self.rmult = 0
        self.prev_lencoder = 0
        self.prev_rencoder = 0
        self.x = 0                  # position in xy plane 
        self.y = 0
        self.th = 0
        self.dx = 0                 # speeds in x/rotation
        self.dr = 0
        self.prev_th = 0
        self.quaternion = Quaternion(0,0,0,1)
        self.then = rospy.Time.now()
        
        # subscriptions
        rospy.Subscriber("lwheel", Int16, self.lwheelCallback)
        rospy.Subscriber("rwheel", Int16, self.rwheelCallback)
        rospy.Subscriber("quat", Quaternion, self.quatCallback)
        
        # Publishers
        self.odomPub = rospy.Publisher("odom", Odometry,queue_size=10)
        self.controlPub = rospy.Publisher("control", Int16, queue_size =10)
        self.odomBroadcaster = TransformBroadcaster()
        
    #############################################################################
    def spin(self):
    #############################################################################
        r = rospy.Rate(self.rate)
        for i in range(30):
            self.controlPub.publish(self.control)
            rospy.Rate(20).sleep()
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
            
    #############################################################################
    def update(self):
    #############################################################################
        th = 0
        now = rospy.Time.now()
        if now > self.t_next:
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()
            
            # calculate odometry
            if self.enc_left == None:
                d_left = 0
                d_right = 0
            else:
                d_left = (self.left - self.enc_left) / self.ticks_meter
                d_right = (self.right - self.enc_right) / self.ticks_meter
            self.enc_left = self.left
            self.enc_right = self.right
           
            # distance traveled is the average of the two wheels 
            d = ( d_left + d_right ) / 2
            
            # this approximation works (in radians) for small angles
            if self.prev_th - asin(self.quaternion.z) * 2 != 0:
                th = asin(self.quaternion.z) * 2 - self.prev_th
                self.prev_th = asin(self.quaternion.z) * 2
                            
            # calculate velocities
            self.dx = d / elapsed
            self.dr = th / elapsed
           
            if (d != 0):
                # calculate distance traveled in x and y
                x = cos( th ) * d
                y = -sin( th ) * d
                # calculate the final position of the robot
                self.x = self.x + ( cos( self.th ) * x - sin( self.th ) * y )
                self.y = self.y + ( sin( self.th ) * x + cos( self.th ) * y )
            if( th != 0):
                self.th = self.th + th
            
            # publish the odom information
            self.odomBroadcaster.sendTransform(
                (self.x, self.y, 0),
                (self.quaternion.x, self.quaternion.y, self.quaternion.z, self.quaternion.w),
                rospy.Time.now(),
                self.base_frame_id,
                self.odom_frame_id
                )
            
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self.odom_frame_id
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = self.quaternion
            odom.child_frame_id = self.base_frame_id
            odom.twist.twist.linear.x = self.dx
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = self.dr
            self.odomPub.publish(odom)

    #############################################################################
    def quatCallback(self, msg):
    #############################################################################
        qArray = [msg.x, msg.y, msg.z, msg.w]/np.linalg.norm([msg.x, msg.y, msg.z, msg.w])
        self.quaternion.x = qArray[0]
        self.quaternion.y = -qArray[1]
        self.quaternion.z = -qArray[2]
        self.quaternion.w = qArray[3]
    
    #############################################################################
    def lwheelCallback(self, msg):
    #############################################################################
        enc = msg.data
        if (enc < self.encoder_low_wrap and self.prev_lencoder > self.encoder_high_wrap):
            self.lmult = self.lmult + 1
            
        if (enc > self.encoder_high_wrap and self.prev_lencoder < self.encoder_low_wrap):
            self.lmult = self.lmult - 1
            
        self.left = 1.0 * (enc + self.lmult * (self.encoder_max - self.encoder_min)) 
        self.prev_lencoder = enc
        
    #############################################################################
    def rwheelCallback(self, msg):
    #############################################################################
        enc = msg.data
        if(enc < self.encoder_low_wrap and self.prev_rencoder > self.encoder_high_wrap):
            self.rmult = self.rmult + 1
        
        if(enc > self.encoder_high_wrap and self.prev_rencoder < self.encoder_low_wrap):
            self.rmult = self.rmult - 1
            
        self.right = 1.0 * (enc + self.rmult * (self.encoder_max - self.encoder_min))
        self.prev_rencoder = enc

#############################################################################
#############################################################################
if __name__ == '__main__':
    """ main """
    diffTf = DiffTfq()
    diffTf.spin()
    
    
   
