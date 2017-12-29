import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

rospy.init_node("demo")

class demo:
    _odom = 0
    def callback(self,odom):
        self._odom = odom.pose.pose.position
    def get_pos(self):
        return self._odom

#build object
pos = demo()

#publish
pub_cmd = rospy.Publisher("RosAria/cmd_vel",Twist,queue_size=1000)

#subcribe
rospy.Subscriber("RosAria/pose",Odometry,pos.callback)

if __name__ == '__main__':
    cmd = Twist()
    cmd.linear.x = 0
    cmd.angular.z = 0
    
    while not rospy.is_shutdown():
        sleep_rate = rospy.Rate(10) # 10 HZ
        sleep_rate.sleep()
        pub_cmd.publish(cmd)    
        print "curent position is",pos.get_pos()
