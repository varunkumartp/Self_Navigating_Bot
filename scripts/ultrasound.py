#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Range

#############################################################################
class ultrasound:
#############################################################################
    
    #############################################################################
    def __init__(self):
    #############################################################################
        rospy.init_node("ultrasound")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        
        # ultrasound 
        self.left_sensor = Range()
        self.left_sensor.header.frame_id = "/ultrasound_left"
        self.left_sensor.radiation_type = 0
        self.left_sensor.field_of_view = 0.523599
        self.left_sensor.min_range = 0.02
        self.left_sensor.max_range = 1.00
        self.right_sensor = Range()
        self.right_sensor.header.frame_id = "/ultrasound_right"
        self.right_sensor.radiation_type = 0
        self.right_sensor.field_of_view = 0.523599
        self.right_sensor.min_range = 0.02
        self.right_sensor.max_range = 1.00
        self.center_sensor = Range()
        self.center_sensor.header.frame_id = "/ultrasound_center"
        self.center_sensor.radiation_type = 0
        self.center_sensor.field_of_view = 0.523599
        self.center_sensor.min_range = 0.02
        self.center_sensor.max_range = 1.00
        
        # subscribers
        rospy.Subscriber("left", Float32, self.leftCallback)
        rospy.Subscriber("center", Float32, self.centerCallback)
        rospy.Subscriber("right", Float32, self.rightCallback)
        
        # publishers
        self.left = rospy.Publisher("range_left", Range, queue_size = 10)
        self.right = rospy.Publisher("range_right", Range, queue_size = 10)
        self.center = rospy.Publisher("range_center", Range, queue_size = 10)
        
    #############################################################################
    def spin(self):
    #############################################################################
        while not rospy.is_shutdown():
            rospy.spin()
    
    #############################################################################
    def leftCallback(self,msg):
    #############################################################################
        self.left_sensor.header.stamp = rospy.Time.now()
        self.left_sensor.range = msg.data
        self.left.publish(self.left)        
    
    #############################################################################
    def rightCallback(self,msg):
    #############################################################################
        self.right_sensor.header.stamp = rospy.Time.now()
        self.right_sensor.range = msg.data
        self.right.publish(self.right)
    
    #############################################################################
    def centerCallback(self,msg):
    #############################################################################
        self.center_sensor.header.stamp = rospy.Time.now()
        self.center_sensor.range = msg.data
        self.center.publish(self.center)
        
#############################################################################
#############################################################################
if __name__ == '__main__':
    """ main """
    us = ultrasound()
    us.spin()
    