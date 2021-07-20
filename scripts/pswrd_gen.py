#!/usr/bin/env python

import pywhatkit
import random
import rospy
from std_msgs.msg import String, Bool
  
#############################################################################
class pswd_gen():
#############################################################################

    #############################################################################
    def __init__(self):
    #############################################################################
        rospy.init_node("pswd_gen")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)  #10260
        self.password = ''
        self.data = '1234567890'
        self.length = 4
        self.phone_book = {
                            "user1" : "+911234567890",  # add mobile number of user1
                            "user2" : "+911234567890",  # add mobile number of user2
                            "admin" : "+911234567890"   # add mobile number of admin
                            }
        
    #############################################################################
    def send_msg(self,msg,contact):
    #############################################################################
        pywhatkit.sendwhatmsg_instantly(contact, msg, wait_time = 10, tab_close = True)
        
    #############################################################################
    def send_pswd(self,contact):
    #############################################################################
        self.password = "".join(random.sample(self.data,self.length))
        self.send_msg(self.password,contact)
        
    #############################################################################
    def update(self):
    #############################################################################
        for i in range(3):
            self.otp = input("Enter the OTP : ")
            if self.otp == '':
                self.otp = input()
            if self.password == self.otp:
                print("Correct OTP")
                self.send_msg('OTP entered correctly',self.phone_book['admin'])
                return
            else:
                print("Incorrect OTP. Chances left : ",2-i)
        print("System Locked")
        self.send_msg('OTP entered incorrectly',self.phone_book['admin'])
        
    #############################################################################
    def spin(self):
    #############################################################################
        self.send_pswd(self.phone_book['user1'])
        self.update() 
        exit()
            
 #############################################################################
 #############################################################################         
if __name__ == "__main__":
    pswd = pswd_gen()
    pswd.spin()