#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String

twist = Twist()
cmd_pub=rospy.Publisher('cmd_vel',Twist,queue_size=1)


def stop():
	twist.linear.x = 0
	twist.angular.z = 0
	cmd_pub.publish(twist)


def move(dist, linear):
	temp = dist/linear
	t0= rospy.Time.now().to_sec()
	time = 0
	while (time<temp):
		twist.linear.x = linear
		twist.angular.z = 0
		cmd_pub.publish(twist)
		t1 = rospy.Time.now().to_sec()
		time = t1- t0
	stop()
	rospy.sleep(0.2)



def turn(angle,angular):
	temp = angle/angular
	t0= rospy.Time.now().to_sec()
	time = 0
	while (time<temp):
		twist.linear.x = 0
		twist.angular.z = angular
		cmd_pub.publish(twist)
		t1 = rospy.Time.now().to_sec()
		time = t1- t0
	stop()
	rospy.sleep(0.2)
	
def twistandturn(dist,linear,angular):
	temp = dist/linear
	t0= rospy.Time.now().to_sec()
	time = 0
	while (time<temp):
		twist.linear.x = linear
		twist.angular.z = angular
		cmd_pub.publish(twist)
		t1 = rospy.Time.now().to_sec()
		time = t1- t0
	stop()



def publisher_node():
	print('TODO: initialize the publisher node here, \
            and publish wheel command to the cmd_vel topic')

	#Seteup of robot

	freq = 100
	rate = rospy.Rate(freq) #10Hz
	time = 0

	linear = .2
	angular = .3
	
	
	#Move forward for 1m
	dist = 1.793
	move(dist, linear)
	
	#Do the curve
	linear = 0.1
	dist = (3*math.pi*0.293)/4
	angular = 3*math.pi / (4 * (dist / linear))
	twistandturn(dist,linear,angular)
	pass
	
	
def main():
	try:
		rospy.init_node('motor')
		publisher_node()
	except rospy.ROSInterruptException:
		pass
    

if __name__ == '__main__':
	main()
