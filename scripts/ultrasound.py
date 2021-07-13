#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, Int16
from sensor_msgs.msg import Range
from tf.broadcaster import TransformBroadcaster

#############################################################################
class ultrasound:
#############################################################################
    
    #############################################################################
    def __init__(self):
    #############################################################################
        rospy.init_node("ultrasound")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)    
        
        self.left = 0
        self.right = 0
        self.center = 0
        self.max = 0.3
        self.side_max = 0.4
        
        # subscribers
        rospy.Subscriber("sonar/left", Float32, self.leftCallback)
        rospy.Subscriber("sonar/center", Float32, self.centerCallback)
        rospy.Subscriber("sonar/right", Float32, self.rightCallback)
    
        # publishers
        self.left_pwm_pub = rospy.Publisher("motors/pwm/lwheel", Int16, queue_size = 10)
        self.right_pwm_pub = rospy.Publisher("motors/pwm/rwheel", Int16, queue_size = 10)
        self.leftPub = rospy.Publisher("range/left", Range, queue_size = 10)
        self.rightPub = rospy.Publisher("range/right", Range, queue_size = 10)
        self.centerPub = rospy.Publisher("range/center", Range, queue_size = 10)
        
    #############################################################################
    def spin(self):
    #############################################################################
        while not rospy.is_shutdown():
            rospy.spin()
    
    #############################################################################
    def check(self):
    #############################################################################
        if self.center < self.max and self.left < self.max and self.right < self.max:
            self.send(128,-128)
        
        elif self.center < self.max:
            if self.left < self.right:
                self.send(128,0)
                rospy.loginfo("right")
            elif self.left > self.right:
                self.send(0,128)
                rospy.loginfo("left")
            
        elif self.left < self.side_max:
            if self.left < self.right:
                self.send(128,0)
                rospy.loginfo("right")
            elif self.left > self.right:
                self.send(0, 0)
                rospy.loginfo("stop")
        
        elif self.right < self.side_max:
            if self.left < self.right:
                self.send(0, 0)
                rospy.loginfo("stop")
            elif self.left > self.right:
                rospy.loginfo("left")
                self.send(0,128)
        else:
            self.send(0, 0)
            rospy.loginfo("stop")
           
    #############################################################################
    def send(self, l, r):
    #############################################################################
        self.left_pwm_pub.publish(l)
        self.right_pwm_pub.publish(r)
    
    #############################################################################
    def leftCallback(self,msg):
    #############################################################################
        usl = TransformBroadcaster()
        usl.sendTransform((0.05 ,0.085, -0.025),
                          (0, 0, 0.258819, 0.9659258),
                          rospy.Time.now(),
                          "ultrasound_left",
                          "base_link")
        left_sensor = Range()
        left_sensor.header.stamp = rospy.Time.now()
        left_sensor.header.frame_id = "ultrasound_left"
        left_sensor.radiation_type = 0
        left_sensor.field_of_view = 0.610865
        left_sensor.min_range = 0.0
        left_sensor.max_range = 1.01
        left_sensor.range = msg.data
        self.left = msg.data
        if self.left < self.side_max:
            self.check()
        self.leftPub.publish(left_sensor)        
    
    #############################################################################
    def rightCallback(self,msg):
    #############################################################################
        usr = TransformBroadcaster()
        usr.sendTransform((0.05 ,-0.085, -0.025),
                          (0, 0, -0.258819, 0.9659258),
                          rospy.Time.now(),
                          "ultrasound_right",
                          "base_link")
        right_sensor = Range()
        right_sensor.header.stamp = rospy.Time.now()
        right_sensor.header.frame_id = "ultrasound_right"
        right_sensor.radiation_type = 0
        right_sensor.field_of_view = 0.610865
        right_sensor.min_range = 0.0
        right_sensor.max_range = 1.01
        right_sensor.range = msg.data
        self.right = msg.data
        if self.right < self.side_max:
            self.check()
        self.rightPub.publish(right_sensor)
    
    #############################################################################
    def centerCallback(self,msg):
    #############################################################################
        usc = TransformBroadcaster()
        usc.sendTransform((0.06 ,0.0, -0.025),
                          (0.0, 0.0, 0.0, 1.0),
                          rospy.Time.now(),
                          "ultrasound_center",
                          "base_link")
        center_sensor = Range()
        center_sensor.header.stamp = rospy.Time.now()
        center_sensor.header.frame_id = "ultrasound_center"
        center_sensor.radiation_type = 0
        center_sensor.field_of_view = 0.610865
        center_sensor.min_range = 0.0
        center_sensor.max_range = 1.01
        center_sensor.range = msg.data
        self.center = msg.data
        if self.center < self.max:
            self.check()
        self.centerPub.publish(center_sensor)
        
#############################################################################
#############################################################################
if __name__ == '__main__':
    """ main """
    us = ultrasound()
    us.spin()
    