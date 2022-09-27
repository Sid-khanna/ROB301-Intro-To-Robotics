#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String

twist = Twist()


def stop(cmd_pub):
	twist.linear.x = 0
       	twist.angular.z = 0
       	cmd_pub.publish(twist)


def move(dist, linear,cmd_pub):
	twist.linear.x = linear
	twist.angular.z = 0
	cmd_pub.publish(twist)
	rospy.sleep(dist/linear)
	stop(cmd_pub)



def turn(angle,angular,cmd_pub):
        twist.linear.x = 0
        twist.angular.z = angular
        cmd_pub.publish(twist)
	rospy.sleep(angle/angular)
	stop(cmd_pub)

def twistandturn(dist,linear,angular,cmd_pub):
	twist.linear.x = linear
        twist.angular.z = angular
        cmd_pub.publish(twist)
	rospy.sleep(dist/linear)
	stop(cmd_pub)




def publisher_node():
    	print('TODO: initialize the publisher node here, \
            and publish wheel command to the cmd_vel topic')

	#Seteup of robot
	cmd_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
	rospy.sleep(1)
	freq = 100
	rate = rospy.Rate(freq) #10Hz
	time = 0

	linear = .1
	angular = .3




	#Move forward for 1m
	dist = 1.793
	move(dist, linear,cmd_pub)

	#Do the curve
	dist = 3*math.pi*0.293/4
	angular = 3*math.pi / (4 * dist / linear)
	twistandturn(dist,linear,angular,cmd_pub)


    	pass


	

def main():
    	try:
        	rospy.init_node('motor')
        	publisher_node()
   	except rospy.ROSInterruptException:
        	pass
    

if __name__ == '__main__':
    	main()
