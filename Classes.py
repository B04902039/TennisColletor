from tf.transformations import euler_from_quaternion
import time
import roslib
import math
import rospy
import cv2
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class robot_location:
    robot_pos = 0
    amcl_pos = 0
    _refrncePt = 0
    _dist = 0
    _turn = 0
    _refrnceAng = 0
    _Flag = True
    
    def odometry(self,odom):
        self.robot_pos = odom.pose.pose
        return self.robot_pos
    
    def mark(self):
        self._refrncePt = self.getEuclidean()
        yaw = self.getOrientation()
        self._refrnceAng = yaw
                               
    def travelDist(self):
        (Ox,Oy) = self._refrncePt
        (Px,Py) = self.getEuclidean()
        self._dist = math.sqrt((Ox-Px)**2 +(Oy-Py)**2)
                
        return self._dist

    def travelAng(self):
        self._refrnceAng
        yaw = self.getOrientation()
        self._turn = yaw - self._refrnceAng 
        return self._turn
    
    def noSpin(self):
        return self._Flag

    def updateSpinStatus(self, sona):
        self._Flag = sona.data
        return self._Flag

    def amcl(self, tmp):
        self.amcl_pos = tmp.pose.pose

    def getOrientation(self):
        (roll,pitch,yaw) = euler_from_quaternion([self.amcl_pos.orientation.x, \
        self.amcl_pos.orientation.y, self.amcl_pos.orientation.z, self.amcl_pos.orientation.w])
        #print yaw
        return yaw

    def getEuclidean(self):
        return (self.amcl_pos.position.x, self.amcl_pos.position.y)
        

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

    #cv2.imshow("Image window", self.cv_image)
    #cv2.waitKey(3)
