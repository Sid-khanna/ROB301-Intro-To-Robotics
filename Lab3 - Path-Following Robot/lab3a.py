#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt32


class Controller(object):
	def __init__(self):
		# publish motor commands
		self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

		# subscribe to detected line index
		self.color_sub = rospy.Subscriber(
		"line_idx", UInt32, self.camera_callback, queue_size=1
		)
		self.actual = 0
		
		

	def camera_callback(self, msg):
		"""Callback for line index."""
		# access the value using msg.data
		self.actual = msg.data
		rospy.loginfo(self.actual)
		pass

	def follow_the_line(self):
		"""
		TODO: complete the function to follow the line
		"""
		actual = self.actual
		r = rospy.Rate(20)
		twist=Twist()
		twist.linear.x = 0.05
		twist.angular.z = 0
		self.cmd_pub.publish(twist)
		t0 = rospy.Time.now().to_sec()
		t = rospy.Time.now().to_sec() - t0
		correction = 0
		desired = 320
		while not rospy.is_shutdown():
			t = rospy.Time.now().to_sec() - t0
			error = desired - self.actual
			if error < 0:
				correction = -0.5
			elif error > 0:
				correction = 0.5
			else:
				correction = 0
			twist.linear.x = 0.05
			twist.angular.z = correction
			self.cmd_pub.publish(twist)
			r.sleep()
		twist.linear.x = 0
		twist.angular.z = 0
		self.cmd_pub.publish(twist)
		pass
        


if __name__ == "__main__":
	rospy.init_node("lab3")
	controller = Controller()
	controller.follow_the_line()
