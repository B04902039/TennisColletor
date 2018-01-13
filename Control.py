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
from tf.transformations import euler_from_quaternion

from Vision import findCentroid, closestBall, image2global, InRange

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray

def PI_control(Cx,Cy,ori_sum,pub_cmd):
    align = False
    ready = False
    cmd = Twist()
  #  try:
    ori_err = 250 - Cx # Cx in range (0,640)
    ori_sum
    ori_sum += ori_err
    
    if abs(ori_err) < 20:
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
        
   # except:
   #     pass

def Fetch(robot, pub_cmd):
    Ready =True
    Finish = False
    cmd = Twist()

    robot.mark()    # mark to go forward
    while robot.travelDist() <= 0.8:
        cmd.linear.x = 0.15
        pub_cmd.publish(cmd)
        time.sleep(0.01)
    
    robot.mark()    # mark to go backward
    while robot.travelDist() <= 0.8:
        cmd.linear.x = -0.15
        pub_cmd.publish(cmd)
        time.sleep(0.01)
    
    return

def spinAround(robot, cx, cy, pub_cmd,ic,origin):
    cmd = Twist()

    robot.mark()    # mark current location
    while cy == -999:   # no ball in vision
        cmd.linear.x = 0.1
        cmd.angular.z = 0.5
        pub_cmd.publish(cmd)    # send command
        time.sleep(0.01)

        (cx, cy) = closestBall(robot,ic,origin) # update vision
        #print"-----------------",robot.travelAng()
        if robot.travelAng() > -0.3 and robot.travelAng() < -0.1: # turn more than one cycle
            time.sleep(1) 
            return True # region clean
    return False


def GoToNext(robot_pos, pub_cmd,path,pub_path):
    cmd = Twist()
    IK_pos= np.array([1.8,1,0,0])
    (roll,pitch,yaw) = euler_from_quaternion(\
    [robot_pos.robot_pos.orientation.x, robot_pos.robot_pos.orientation.y, robot_pos.robot_pos.orientation.z, robot_pos.robot_pos.orientation.w])
    
    x_err = IK_pos[0] - robot_pos.robot_pos.position.x
    y_err = IK_pos[1] - robot_pos.robot_pos.position.y
    total_err = math.sqrt(x_err**2 +y_err**2) 
    alpha = math.atan2(y_err,x_err)     
    ori_err = alpha - yaw
    #global ori_sum
    #ori_sum[0] += ori_err
            
    
    print"\n\n\nTURN\n\n\n"
    while (not(abs(ori_err) < 0.05)) and (not(x_err < 0.5 and abs(y_err) < 0.3)): # turn
        append(robot_pos,pub_path,path)
        print("\nyaw, ori_rr= "+str(yaw)+" "+str(ori_err))        
        (roll,pitch,yaw) = euler_from_quaternion(\
        [robot_pos.robot_pos.orientation.x, robot_pos.robot_pos.orientation.y, robot_pos.robot_pos.orientation.z, robot_pos.robot_pos.orientation.w])
        x_err = IK_pos[0] - robot_pos.robot_pos.position.x
        y_err = IK_pos[1] - robot_pos.robot_pos.position.y
        alpha = math.atan2(y_err, x_err)     
        ori_err = alpha - yaw
        angular = ori_err #+ 0.0015*ori_sum[0]
        if angular > 0.25:
            angular = 0.2
        elif angular < -0.25:
            angular = -0.2
        
        #cmd.linear.x = 0.1
        cmd.angular.z = angular
        pub_cmd.publish(cmd)
        time.sleep(0.01)
    print"\n\n\nForward\n\n\n"    
    while not (x_err < 0.5 and abs(y_err) < 0.3): # linear approching
        print("\nxerr, yerr= "+str(x_err)+' '+str(y_err))
        print("\nyaw, ori_rr= "+str(yaw)+" "+str(ori_err))
        append(robot_pos,pub_path,path)
        (roll,pitch,yaw) = euler_from_quaternion(\
        [robot_pos.robot_pos.orientation.x, robot_pos.robot_pos.orientation.y, robot_pos.robot_pos.orientation.z, robot_pos.robot_pos.orientation.w])
        x_err = IK_pos[0] - robot_pos.robot_pos.position.x
        y_err = IK_pos[1] - robot_pos.robot_pos.position.y
        total_err = math.sqrt(x_err**2 +y_err**2) 
        alpha = math.atan2(y_err,x_err)     
        ori_err = alpha - yaw
        #global ori_sum
        #ori_sum[0] -= ori_err               
        
        velocity = 0.3*(total_err)#*math.cos(ori_err)
        if velocity > 0.2:
            velocity = 0.2
        elif velocity < -0.2:
            velocity = -0.2

        angular = 0.5*ori_err #+ 0.0015*ori_sum[0]
        if angular > 0.25:
            angular = 0.2
        elif angular < -0.25:
            angular = -0.2

        cmd.linear.x = velocity
        cmd.angular.z = 0
        
        
        #print"goal =  ",IK_pos
        #doc.write("\nalpha, ori_err =  "+str(alpha)+" "+str(ori_err))
        pub_cmd.publish(cmd)
        time.sleep(0.01)
    #print"\n\n\nTURN\n\n\n"
    while not (abs(yaw) < 0.05): # turn
        print("\nyaw = "+str(yaw))
        append(robot_pos,pub_path,path)
        (roll,pitch,yaw) = euler_from_quaternion(\
        [robot_pos.robot_pos.orientation.x, robot_pos.robot_pos.orientation.y, robot_pos.robot_pos.orientation.z, robot_pos.robot_pos.orientation.w])

        angular = -0.5*yaw #+ 0.0015*ori_sum[0]
        if angular > 0.25:
            angular = 0.2
        elif angular < -0.25:
            angular = -0.2
        cmd.linear.x = 0.05
        cmd.angular.z = angular
        pub_cmd.publish(cmd)
        time.sleep(0.01)
    #print"\n\n\nNEXT BLOCK\n\n\n"
    

def append(robot,pub_path,path):
    #print "\nthe appended pos is \n"+str(IK_pos)
    pose = PoseStamped()
    pose.pose.position.x = robot.robot_pos.position.x
    pose.pose.position.y = robot.robot_pos.position.y
    
    path.header.frame_id="laser"
    path.header.stamp=rospy.Time.now()
    path.poses.append(pose)
    pub_path.publish(path)


