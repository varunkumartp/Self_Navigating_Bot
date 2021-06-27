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
        left_sensor = Range()
        left_sensor.header.stamp = rospy.Time.now()
        left_sensor.header.frame_id = "/ultrasound_left"
        left_sensor.radiation_type = 0
        left_sensor.field_of_view = 0.523599
        left_sensor.min_range = 0.02
        left_sensor.max_range = 1.00
        left_sensor.range = msg.data
        self.left.publish(left_sensor)        
    
    #############################################################################
    def rightCallback(self,msg):
    #############################################################################
        right_sensor = Range()
        right_sensor.header.stamp = rospy.Time.now()
        right_sensor.header.frame_id = "/ultrasound_right"
        right_sensor.radiation_type = 0
        right_sensor.field_of_view = 0.523599
        right_sensor.min_range = 0.02
        right_sensor.max_range = 1.00
        right_sensor.range = msg.data
        self.right.publish(right_sensor)
    
    #############################################################################
    def centerCallback(self,msg):
    #############################################################################
        center_sensor = Range()
        center_sensor.header.stamp = rospy.Time.now()
        center_sensor.header.frame_id = "/ultrasound_center"
        center_sensor.radiation_type = 0
        center_sensor.field_of_view = 0.523599
        center_sensor.min_range = 0.02
        center_sensor.max_range = 1.00
        center_sensor.range = msg.data
        self.center.publish(center_sensor)
        
#############################################################################
#############################################################################
if __name__ == '__main__':
    """ main """
    us = ultrasound()
    us.spin()
    