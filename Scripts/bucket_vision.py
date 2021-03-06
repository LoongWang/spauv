#!/usr/bin/env python

import roslib 
import rospy 
import os
import sys
import cv2
import cv2.cv
from cv_bridge import CvBridge, CvBridgeError
import actionlib
from dynamic_reconfigure.server import Server as DynServer

#from bbauv_msgs.msg import *
#from bbauv_msgs.srv import *
import spauv.cfg.bucketConfig as Config
from sensor_msgs.msg import Image

import signal
#from collections import deque

import numpy as np

class BucketDetector:
    #HSV thresholds for red color
    lowThresh1 = np.array([ 92, 0, 10 ])
    hiThresh1 = np.array([ 131, 255, 245 ]) 
    areaThresh = 30000
    '''
    params = {'hueHigh': 255, 
            'satLow':0, 'satHigh': 255,
            'valLow':0, 'valHigh': 255,
            'hueLow':0}
    '''
    def __init__(self):
        self.image_pub = rospy.Publisher("/image_filtered",Image)
        cv2.namedWindow("Image window", 1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image_raw",Image,self.frontcam_callback)
        self.dyn_reconf_server = DynServer(Config, self.reconfigure)
        
    def frontcam_callback(self,ros_image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV) #Convert to HSV image
        #Perform red thresholding
        contourImg = cv2.inRange(hsv_image, self.lowThresh1, self.hiThresh1)
        cv2.imshow("Image window", contourImg)
        cv2.waitKey(30)
        #hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # define range of blue color in HSV
        #lower_blue = np.array([110,50,50])
        #upper_blue = np.array([130,255,255])
        
        '''    
        (rows,cols,channels) = cv_image.shape
        if cols > 60 and rows > 60 :
            cv2.circle(cv_image, (50,50), 10, 255)
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)
        '''
        
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)
            
    def reconfigure(self, config, level):
        rospy.loginfo("Got reconfigure request!")
        self.lowThresh1[0] = config['hueLow']
        self.lowThresh1[1] = config['satLow']
        self.lowThresh1[2] = config['valLow']

        self.hiThresh1[0] = config['hueHigh']
        self.hiThresh1[1] = config['satHigh']
        self.hiThresh1[2] = config['valHigh']

        self.areaThresh = config['area_thresh']

        return config
    
    
'''def main(args):
    ic = BucketDetector()
    rospy.init_node('BucketDetector', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows() '''


if __name__ == "__main__":
    rospy.init_node("bucket_vision")
    bucketDector = BucketDetector()
    rospy.spin()
