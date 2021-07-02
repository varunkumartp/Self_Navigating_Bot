#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
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
        
        # subscribers
        rospy.Subscriber("ultrasound/left", Float32, self.leftCallback)
        rospy.Subscriber("ultrasound/center", Float32, self.centerCallback)
        rospy.Subscriber("ultrasound/right", Float32, self.rightCallback)
        
        # publishers
        self.left = rospy.Publisher("range/left", Range, queue_size = 10)
        self.right = rospy.Publisher("range/right", Range, queue_size = 10)
        self.center = rospy.Publisher("range/center", Range, queue_size = 10)
        
    #############################################################################
    def spin(self):
    #############################################################################
        while not rospy.is_shutdown():
            rospy.spin()
    
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
        self.left.publish(left_sensor)        
    
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
        self.right.publish(right_sensor)
    
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
        self.center.publish(center_sensor)
        
#############################################################################
#############################################################################
if __name__ == '__main__':
    """ main """
    us = ultrasound()
    us.spin()
    