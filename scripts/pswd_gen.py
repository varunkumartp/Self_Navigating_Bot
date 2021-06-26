#!/usr/bin/env python

import pywhatkit
import random
import rospy
from std_msgs.msg import String, Bool
  
class pswd_gen():
    def __init__(self):
        rospy.init_node("pswd_gen")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)  #10260
        self.password = ''
        self.data = '1234567890'
        self.length = 4
        self.phone_book = {
                            "user1" : "+918310726474",
                            "user2" : "+919036780663",
                            "admin" : "+919448451533" 
                            }

        # Publisher
        self.pswd = rospy.Publisher('otp', String, queue_size = 10)
        
        # Subscriber
        rospy.Subscriber('status',Bool, self.callback)           
    
    def send_msg(self,msg,contact):
        pywhatkit.sendwhatmsg_instantly(contact, msg, wait_time = 10, tab_close = True)
        
    def send_pswd(self,contact):
        self.password = "".join(random.sample(self.data,self.length))
        self.send_msg(self.password,contact)
        self.pswd.publish(self.password)  
        
    def callback(self,msg):
        if(msg):
            self.send_msg('OTP entered correctly',self.phone_book['admin'])
            self.send_pswd(self.phone_book['user2'])
        else:
            self.send_msg('OTP entered incorrectly',self.phone_book['admin'])
    
    def spin(self):
        self.send_pswd(self.phone_book['user1'])
        while not rospy.is_shutdown():
            rospy.spin()
       
if __name__ == "__main__":
    pswd = pswd_gen()
    pswd.spin()