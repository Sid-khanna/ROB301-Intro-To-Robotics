#!/usr/bin/env python
from pickle import FALSE
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import UInt32

import matplotlib.pyplot as plt
import math
import numpy as np


class KalmanFilter(object):
    def __init__(self, h, d, x_0, Q, R, P_0):
        self.h = h
        self.d = d

        self.Q = Q	# variance of the state noise
        self.R = R	# variance of the measurement noise
        self.S = 0	# measurement covariance
        self.P = P_0	# State covariance
        self.x = x_0	# state variable
        self.W = 0	# Kalman Gain
        self.D = 0

        self.u = 0  # initialize the cmd_vel input
        self.phi = np.nan  # initialize the measurement input
        self.phip = 0
        
        self.x_list = []
        self.P_list = []

        self.dt = 0.0
        self.time = rospy.Time.now().to_sec()

        self.state_pub = rospy.Publisher("state", Float64, queue_size=1)
        self.scan_sub = rospy.Subscriber(
            "scan_angle", Float64, self.scan_callback, queue_size=1
        )
        self.cmd_sub = rospy.Subscriber("cmd_vel_noisy", Twist, self.cmd_callback)

    def cmd_callback(self, cmd_msg):
        self.u = cmd_msg.linear.x

    ## updates self.phi with the most recent measurement of the tower.
    def scan_callback(self, msg):
        self.phi = msg.data

    ## call within run_kf to update the state with the measurement
    def predict(self, u=0):
        self.dt = rospy.Time.now().to_sec() - self.time  # calculate B
        self.x = self.x + self.u*self.dt                 # a priori state estimate
        self.time = rospy.Time.now().to_sec()            # set new time for next iteration
        self.P = self.P + self.Q                         # a priori state covariance estimate
        self.D= h/(h**2 + (d-self.x)**2)                 # calculate D
        self.S = self.D*self.P*self.D+self.R             # measurement covariance prediction
        self.W = self.P*self.D/self.S                    # update Kalman Gain
        self.P = self.P - self.W*self.S*self.W           # update state covariance
        self.P_list.append(self.P)                       # add to list so that we can plot later

        """
        TODO: update state via the motion model, and update the covariance with the process noise
        """
        return

    ## call within run_kf to update the state with the measurement
    def measurement_update(self):
        self.phip = math.atan(h/(self.d-self.x))
        """
        TODO: update state when a new measurement has arrived using this function
        """
        return

    def run_kf(self):
        current_input = self.u
        current_measurement = self.phi
        if math.isnan(current_measurement) == False:
            self.measurement_update()
            self.predict()
            self.x = self.x + self.W*(self.phi - self.phip)
            self.x_list.append(self.x)
        else:
            self.predict()
            self.x_list.append(self.x)
        self.state_pub.publish(self.x)
        
        """
        TODO: complete this function to update the state with current_input and current_measurement
        """
        
if __name__ == "__main__":
    rospy.init_node("lab4")

    h = 0.60  # y distance to tower
    d = 0.60 * 3  # x distance to tower (from origin)

    x_0 = 0  # initial state position

    Q = 1  # TODO: Set process noise covariance
    R = 1  # TODO: measurement noise covariance
    P_0 = 1  # TODO: Set initial state covariance

    kf = KalmanFilter(h, d, x_0, Q, R, P_0)
    rospy.sleep(1)

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        kf.run_kf()
        rate.sleep()
    x = np.array(kf.x_list)
    P = np.array(kf.P_list)
    plt.plot(x)
    plt.show()
    plt.plot(P)
    plt.show()
