#!/usr/bin/env python
"""Add noise to cmd_vel for use in simulation."""
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np


class NoiseMaker(object):
    def __init__(self, mu=0, std=1e-5):
        self.mu = mu
        self.std = std

        self.cmd_sub = rospy.Subscriber("cmd_vel", Twist, self.cmd_callback)
        self.cmd_pub = rospy.Publisher("cmd_vel_noisy", Twist, queue_size=1)

    def cmd_callback(self, msg):
        # noisy twist is the same but with Gaussian noise on linear x
        twist = msg
        twist.linear.x += np.random.normal(loc=self.mu, scale=self.std)
        self.cmd_pub.publish(twist)


if __name__ == "__main__":
    rospy.init_node("noisy_vel_publisher")

    mu_noise = 0.0  # mean
    std_noise = 0.01  # standard deviation
    noise_maker = NoiseMaker(mu=mu_noise, std=std_noise)
    rospy.spin()
