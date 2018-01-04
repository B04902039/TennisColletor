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


class robot_location:
    robot_pos = 0
    _refrncePt = 0
    _dist = 0
    _turn = 0
    _refrnceAng = 0
    
    def odometry(self,odom):
        self.robot_pos = odom.pose.pose
        return self.robot_pos

    def mark(self):
        self._refrncePt = self.robot_pos      
        (roll,pitch,yaw) = euler_from_quaternion([self.robot_pos.orientation.x, \
        self.robot_pos.orientation.y, self.robot_pos.orientation.z, self.robot_pos.orientation.w])
        self._refrnceAng = yaw    
                               
    def travelDist(self):
        (Ox,Oy) = self._refrncePt.position.x, self._refrncePt.position.y
        (Px,Py) = self.robot_pos.position.x,self.robot_pos.position.y
        self._dist = math.sqrt((Ox-Px)**2 +(Oy-Py)**2)
                
        return self._dist

    def travelAng(self):
        self._refrnceAng
        (roll,pitch,yaw) = euler_from_quaternion([self.robot_pos.orientation.x, \
        self.robot_pos.orientation.y, self.robot_pos.orientation.z, self.robot_pos.orientation.w])
        self._turn = yaw - self._refrnceAng 
        return self._turn

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
