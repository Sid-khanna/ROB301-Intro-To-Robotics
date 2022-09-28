#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String

PI = 3.1415926535897

def main():	
    rospy.init_node("motor_node")
    
    r=rospy.Rate(10)
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    twist=Twist()
    twist.linear.x=0.2
    twist.angular.z=0
    
    t0= rospy.Time.now().to_sec()
    distance = 0
    while (distance <1):
        cmd_pub.publish(twist)
        t1 = rospy.Time.now().to_sec()
        distance = 0.2*(t1-t0)
        r.sleep()
    	
    twist.linear.x=0
    cmd_pub.publish(twist)
    
    twist.angular.z= (90)*2*PI/360
    t0 = rospy.Time.now().to_sec()
    angle = 0
    while (angle< 2*PI):
        cmd_pub.publish(twist)
        rospy.loginfo(twist)
        t1 = rospy.Time.now().to_sec()
        angle = (90)*2*PI/360*(t1-t0)
        r.sleep()
    	
    twist.angular.z=0
    cmd_pub.publish(twist)
    
if __name__ == "__main__":
    main()
