from tf.transformations import euler_from_quaternion
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

def InRange(Cx,Cy,robot,origin,blockMode):
    if blockMode == 0:
        return True 
    (x,y) =  image2global(Cx,Cy)
    #print "\robot position x, y",robot.robot_pos.position.x,robot.robot_pos.position.y
    #print "\n Cx, Cy = ",Cx,Cy
    #print "\n Observed x,y = ",x,y
    observed = np.array([0.01*x,0.01*y,0,1])
    # USE ODEMETRY(robot_pos) to calc homogneous transformaiton matrix     
    yaw = robot.getOrientation()
    (Px,Py) = robot.getEuclidean()
    Pz = 0
    
    HT = np.array([[math.cos(yaw),-math.sin(yaw),0,Px],[math.sin(yaw),math.cos(yaw),0,Py],[0,0,1,Pz],[0,0,0,1]])   
    IK_pos = np.dot(HT,observed)
        
    x_pos = IK_pos[0]-origin[0]
    y_pos = IK_pos[1]-origin[1]
    #print"global frame x,y = ",x_pos,y_pos
            
    
    if x_pos < 2.2 and -0.2 < x_pos and -2.0 < y_pos and y_pos < 0.3:
        return True
    else:
        return False

def findCentroid(image):
    circles = []
    im2, contours, hier = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for i in contours:
        M = cv2.moments(i)
        if M["m00"]>300:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            circles.append([cx,cy])
    return circles

def euclidean_dist(a, b):
    return (a[0]-b[0])**2 + 5*((a[1]-b[1])**2)

def closestBall(robot, ic, origin, blockMode, last=(250,480)): # last is a static var!!!!!!
    # robot: robot location instance, ic: image converter instance, last: the closest ball in last image
    (Cx,Cy,radius) = 250,-999,0
    ic.cv_image = np.array(ic.cv_image)
    output = ic.cv_image.copy()

    # try:
    img_hsv = cv2.cvtColor(ic.cv_image, cv2.COLOR_BGR2HSV)
    img_hsv = cv2.GaussianBlur(img_hsv, (7,7), 0)
    green_low = (55-25, 50, 50)
    green_high = (55+35, 250, 250)
    image = cv2.inRange(img_hsv, green_low, green_high)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
    image = cv2.erode(image, kernel, iterations=1)
    image = cv2.dilate(image, kernel, iterations=1)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
    image = cv2.erode(image, kernel, iterations=2)
    image = cv2.dilate(image, kernel, iterations=2)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11,11))
    image = cv2.erode(image, kernel, iterations=1)
    image = cv2.dilate(image, kernel, iterations=1)
    cv2.imshow("haha",image)
    
    circles = findCentroid(image)
    
    min_dist = 21000000
    for (x, y) in circles:
    #origin = np.array([0,0]) # hardcoding, need to be modified
        if InRange(x,y,robot,origin,blockMode)==True and y>100:
            if euclidean_dist((x,y), last) < min_dist:
                min_dist = euclidean_dist((x,y), last)
                (Cx,Cy) = (x,y)
    radius = 10
    cv2.circle(output, (Cx, Cy), radius, (0, 255, 0), 4)
    #print "the x, y coordinate of the closest circle is",Cx,Cy        
    cv2.imshow("output",output)

    cv2.waitKey(1)
    last = (Cx, Cy)
    return Cx,Cy

def image2global(Cx,Cy):
    dx_Paras = np.array([ -1.09259259e-04,   6.42777778e-02,  -1.31933333e+01,   1.11640000e+03])
    y_Paras  = np.array([ -1.16990257e-05,   1.17341647e-02,  -4.26415158e+00,   6.55088243e+02])
    
    Y_Global = y_Paras[0]*Cy**3 + y_Paras[1]*Cy**2+y_Paras[2]*Cy+y_Paras[3]
    
    delta_x =  dx_Paras[0]*Y_Global**3 + dx_Paras[1]*Y_Global**2+dx_Paras[2]*Y_Global+dx_Paras[3]
    X_Global = (Cx-291)/delta_x*60 - 10
    
    X_car = Y_Global     
    Y_car = -X_Global
    
    return X_car, Y_car