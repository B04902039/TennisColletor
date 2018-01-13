# -*- coding: utf-8 -*-

#!/usr/bin/env python
import time
import roslib
import math
import rospy
import cv2
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

from Classes import robot_location, image_converter
from Vision import findCentroid, closestBall, image2global, InRange
from Control import PI_control, Fetch, spinAround, GoToNext

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray

if __name__ == '__main__':
    robot = robot_location()  
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)

    rospy.Subscriber("/camera/rgb/image_color",Image,ic.callback)
    rospy.Subscriber('/RosAria/pose', Odometry, robot.odometry)
    path = Path() 
    time.sleep(1)    
  
    pub_cmd = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=1000)
    pub_path = rospy.Publisher('/path', Path, queue_size=1)
    
    global ori_sum ; ori_sum = 0
    (Cx,Cy,radius) = 0,0,0
    Ready = False
    cleaned = False
    origin = np.array([0,0])
    last = (250,480)
    
    while not rospy.is_shutdown():
        time.sleep(0.01)
        #try:
        (Cx,Cy)=  closestBall(robot,ic,origin,last)
        if Cy > 100: 
            last = (Cx,Cy)
        #except:
        #    no ball in vision: (Cx,Cy) = (250,-999)
        
        if Cy == -999:	# no ball in vision
            if cleaned is not True:
                cleaned = spinAround(robot, Cx, Cy, pub_cmd,ic,origin)
            elif cleaned == True:
                print"I see no ball"
                cleaned = False
                GoToNext(robot,pub_cmd,path,pub_path)
                origin[0]+=1.8
        elif Cy > 0:
            print"see ball"        
            if Ready == False:
                Ready = PI_control(Cx,Cy,ori_sum,pub_cmd)                    
            elif Ready == True:
                print "Fetch"
                Fetch(robot, pub_cmd)
                last = (250,480)
                Ready = False

        
# x right
# y down
# z outward


    
    
    
    
