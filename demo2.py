import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import time 


rospy.init_node("demo")

class demo:
    _odom = 0
    _refrnceAng = 0
    _new =0 
    def callback(self,odom):
        self._odom = odom.pose.pose.orientation
        return self._odom

    def original_pos(self):
        (roll,pitch,yaw)=euler_from_quaternion([self._odom.x, \
        self._odom.y, self._odom.z, self._odom.w])
        self._refrnceAng=yaw

    def get_current(self):
        (roll,pitch,yaw)= euler_from_quaternion([self._odom.x, \
        self._odom.y, self._odom.z, self._odom.w])
        return yaw
   

#build object
pos = demo()

#publish
pub_cmd = rospy.Publisher("RosAria/cmd_vel",Twist,queue_size=1000)

#subcribe
rospy.Subscriber("RosAria/pose",Odometry,pos.callback)

if __name__ == '__main__':
    cmd = Twist()
    cmd.linear.x = 0
    cmd.angular.z = 5
    time.sleep(1)
    pos.original_pos()        
    
    while not rospy.is_shutdown():
        sleep_rate = rospy.Rate(10) # 10 HZ
        sleep_rate.sleep()
        pub_cmd.publish(cmd)
        odom =pos.get_current() 
        if (odom - pos._refrnceAng )>=3.14/2:
            cmd.angular.z=0     
        print "curent position is",odom
