# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""
#!/usr/bin/env python
import time
import roslib
import sys
import rospy
import cv2
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class image_converter:
  
  def __init__(self):

    self.bridge = CvBridge()
    self.cv_image = ([0])
    self.depth = np.array([0])
  def callback(self,data):
    try:
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    cv2.imshow("Image window", self.cv_image)
    cv2.waitKey(3)


def PI_control(Cx,Cy):
    align = False
    ready = False
    cmd = Twist()
    try:
        ori_err = 320 - Cx # Cx in range (0,640)
        global ori_sum
        ori_sum += ori_err
        
        if abs(ori_err) < 10:
            ori_err = 0
            ori_sum = 0
            align = True
               
        angular = 0.002*ori_err + 0.00001*ori_sum
        if angular > 0.5:
                angular = 0.5
        elif angular < -0.5:
    		    angular = -0.5
        cmd.angular.z = angular
            
        # get closer
        if align == True:
            if Cy < 430:
                cmd.linear.x = 0.1
            elif Cy > 430:
                ready = True
        
        pub_cmd.publish(cmd)
        return ready
        
    except:
        pass

def Fetch():
    Ready =True
    global forward
    forward += 1
    '''
    cmd = Twist()
    cmd.linear.x = 0.2
    
    pub_cmd.publish(cmd)
    '''
    if forward > 10:
        forward = 0
        Ready = False
    return Ready


def closestBall(Cx,Cy,radius):
    (Cx,Cy,radius) = 320,-999,0
    ic.cv_image = np.array(ic.cv_image)
    try:
        image = cv2.inRange(ic.cv_image , (0,70,70), (80,250,250))
        output = ic.cv_image.copy()
        kernel = np.ones((5,5), np.uint8)
        #kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5, 5)) 
        
        image = cv2.erode(image, kernel, iterations=1)#3
        image = cv2.dilate(image, kernel, iterations=1)         
        #cv2.imshow("seg",image)
        circles = cv2.HoughCircles(image, cv2.cv.CV_HOUGH_GRADIENT, 10,40,10,32,30)             
        circles = np.round(circles[0, :]).astype("int")

        for (x, y, r) in circles:
            if r < 20 and y > Cy and y > 250:
                Cy = y
                Cx = x
                radius = r
        cv2.circle(output, (Cx, Cy), radius, (0, 255, 0), 4)
        #print "the x, y coordinate of the closest circle is",Cx,Cy        
        #cv2.imshow("output",output)
        #cv2.waitKey(0.1)
        return Cx,Cy                
    except:
        pass
    
if __name__ == '__main__':
    
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)

    rospy.Subscriber("/camera/rgb/image_color",Image,ic.callback)
    pub_cmd = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=1000)
    global ori_sum ; ori_sum = 0
    global forward; forward = 0
    (Cx,Cy,radius) = 0,0,0
    Ready = False
    Blank = 0
    
    while not rospy.is_shutdown():
        time.sleep(0.01)
        (Cx,Cy)=  closestBall(Cx,Cy,radius) if closestBall(Cx,Cy,radius) else (320,-999)
        print"Cx,Cy=",Cx,Cy
        
        if Cy == -999:
            Blank += 1
            if Blank > 30:
                print "random walk!"
                 
        elif Cy > 0:        
            if Ready == False:
                Ready = PI_control(Cx,Cy)
            elif Ready == True:
                Ready = Fetch()
                print "Fecth!"
            Blank = 0


# x right
# y down
# z outward


    
    
    
    
