#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import numpy as np


class ScanAnglePublisher(object):
    def __init__(self):
        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.scan_cb)
        self.angle_pub = rospy.Publisher("scan_angle", Float64, queue_size=1)

    def scan_cb(self, msg):
        # only use the scan where y >= 0 (i.e., to the left side of the robot)
        in_range = np.array(msg.ranges[0:180])
        angles = np.deg2rad(np.arange(0, 180))

        # computes the y-distance to each measurement
        y_range = in_range * np.sin(angles)

        # filter out measurements with ranges outside of [0.1, 2]m or y_range
        # outside of [0.5, 0.65]m
        # note that the range is 0 when no valid range is obtained: this is
        # filtered out by the 0.1m lower bound
        angles = angles[
            (0.5 <= y_range) & (y_range <= 0.65) & (0.1 <= in_range) & (in_range <= 2)
        ]

        # if no valid measurements, don't publish anything
        if len(angles) == 0:
            return

        # if we have multiple measurements (i.e. multiple lasers hit one or
        # more objects), take the median
        angle = np.median(angles) +0.055

        # publish the message
        angle_msg = Float64()
        angle_msg.data = angle
        self.angle_pub.publish(angle_msg)
        rospy.loginfo(angle)


def main():
    rospy.init_node("scan_angle_publisher")
    pub = ScanAnglePublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
