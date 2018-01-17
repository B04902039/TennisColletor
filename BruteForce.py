import rospy

from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion

import time 

import numpy

rospy.init_node("demo")


class demo:

    _odom = 0

    _refrnceAng = 0
   
    _refrnceX =0

    _new =0 

    def callback(self,odom):

        self._odoma = odom.pose.pose.orientation
        self._odoml = odom.pose.pose.position

        return self._odoma, self._odoml



    def original_pos(self):

        (roll,pitch,yaw)=euler_from_quaternion([self._odoma.x, \

        self._odoma.y, self._odoma.z, self._odoma.w])

        self._refrnceX=self._odoml.x
        self._refrnceAng = yaw
        return self._refrnceX, self._refrnceAng


    def get_current(self):

        (roll,pitch,yaw)= euler_from_quaternion([self._odoma.x, \

        self._odoma.y, self._odoma.z, self._odoma.w])

        return self._odoml.x, yaw

#build object

pos = demo()

#publish

pub_cmd = rospy.Publisher("RosAria/cmd_vel",Twist,queue_size=1000)




#subcribe

rospy.Subscriber("RosAria/pose",Odometry,pos.callback)



if __name__ == '__main__':

    index=0

    cmd = Twist()

    cmd.linear.x = 0.2

    cmd.angular.z = 0
 
    time.sleep(1)

    pos.original_pos()


    while not rospy.is_shutdown():

        sleep_rate = rospy.Rate(10) # 10 HZ

        sleep_rate.sleep()

        pub_cmd.publish(cmd)

        odom =pos.get_current() 


	if index%4==0 : 
            while not abs(odom[0]-pos._refrnceX)>=3.6: #or\
		  # (odom[0]-pos._refrnceX)<=-0.2  :
		   sleep_rate = rospy.Rate(10) # 10 HZ
       		   sleep_rate.sleep()
        	   pub_cmd.publish(cmd)
        	   odom =pos.get_current() 
                   a= (odom[0]-pos._refrnceX)
                   print "linear0", a
      	           cmd.linear.x=0.2
	           cmd.angular.z=0
	    index+=1

	if index%4==2:
		while not (odom[0]-pos._refrnceX)<=0:
		    sleep_rate.sleep()
		    pub_cmd.publish(cmd)
		    odom=pos.get_current()
		    a=(odom[0]-pos._refrnceX)
                    
		    print "linear2",a
		    cmd.linear.x=0.2
		    cmd.angular.z=0
		index+=1
		print index

	if index%4==1:
		while not abs(odom[1] - pos._refrnceAng) >=2.87: #and \
		    #abs((odom[0] - pos._refrnceX))>0.2:
		    sleep_rate = rospy.Rate(10) # 10 HZ
       		    sleep_rate.sleep()		       	
        	    pub_cmd.publish(cmd)
        	    odom =pos.get_current() 
		        
		    cmd.linear.x=0.1		        
	            cmd.angular.z=0.6
                    a= (odom[0]-pos._refrnceX)
    
		    print "lefttrun+Angle",(odom[1]-pos._refrnceAng)
		    print "index",index
		    print "distance%", a
		    
		index+=1
		print index
		
		    
	if index%4==3:
		while not (odom[1]-pos._refrnceAng ) <=0.2: #and \
		    #abs((odom[0] - pos._refrnceX))>0.2:
		    sleep_rate = rospy.Rate(10) # 10 HZ
		    sleep_rate.sleep()		       	 

		    pub_cmd.publish(cmd)
                    odom =pos.get_current() 
		    cmd.linear.x=0.1
		    cmd.angular.z=-0.6
		    a= (odom[0]-pos._refrnceX)

                    print "rightrun+angle",(odom[1]-pos._refrnceAng)
		    print "index",index
		    print "distance%", a
		index+=1
	    
#	if abs((odom[0] - pos._refrnceX))<=0.2 :
#	    cmd.linear.x=0.1
#	    cmd.angular.z=-0.5
	if index>=20:
		cmd.linear.x=0
		cmd.angular.z=0
		index=False
        print "original position is:",pos._refrnceX,pos._refrnceAng
        print "curent position is",odom
	print "index",index
