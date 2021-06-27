#!/usr/bin/env python

import serial
import rospy 
from std_msgs.msg import Float32

#############################################################################
class gps:
#############################################################################
   
    #############################################################################
    def __init__(self):
    #############################################################################
        rospy.init_node("gps")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename) 
        self.rate = rospy.get_param('~rate', 10)
        
        # serial port
        self.serialPort = serial.Serial(port = "/dev/ttyAMA1", baudrate=9600, bytesize=8, timeout=2)
        
        # publishers
        self.latPub =rospy.Publisher('lat', Float32, queue_size = 10)
        self.lonPub =rospy.Publisher('lon', Float32, queue_size = 10)
         
    #############################################################################
    def spin(self):
    #############################################################################
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.get_gps()
            r.sleep()

    #############################################################################
    def get_gps(self):
    #############################################################################
        if(self.serialPort.in_waiting > 0):
            a = self.serialPort.readline().decode('ascii', errors='replace')
            for line in a.split('\n') :
                if line.startswith( '$GPGGA' ) :
                    lat, _, lon = line.strip().split(',')[2:5]
                    lat = round(float(lat)/100,6)
                    lon = round(float(lon)/100,6) 
                    self.latPub.publish(lat)
                    self.lonPub.publish(lon)
                    
#############################################################################
#############################################################################
if __name__ == '__main__':
    GPS = gps()
    GPS.spin()
                    