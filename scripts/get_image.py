#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class getImage():
    def __init__(self):
        rospy.init_node("Display",anonymous=True)
        rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
        
    def spin(self):
        rospy.spin()

    def callback(self,data):
        bridge = CvBridge()
        try:
            cv2_img = bridge.imgmsg_to_cv2(data, "bgr8")
        except (CvBridgeError, e):
            print(e)
        else:
            # Save your OpenCV2 image as a jpeg 
            cv2.imshow("frame",cv2_img)
            cv2.waitKey(1)


def main():
    img = getImage()
    img.spin()


if __name__ == "__main__":
    main()
