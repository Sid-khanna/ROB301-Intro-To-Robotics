#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String, UInt32MultiArray
import numpy as np
import colorsys
import matplotlib
from matplotlib import pyplot as plt
class BayesLoc:
def __init__(self, p0, colour_codes, colour_map, states, twist):
# attributes for ROS nodes
self.colour_sub = rospy.Subscriber(
"mean_img_rgb", UInt32MultiArray, self.colour_callback
)
self.line_sub = rospy.Subscriber("line_idx", String, self.line_callback)
self.twist = twist
self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
# attributes for bayesian localization
self.colour_codes = colour_codes
self.colour_map = colour_map
self.states = states
self.probability = p0
# attributes for PID control and colour detection
self.direction = 1 #start by going forward on track
self.line_index =0
self.prev_line_index = 0
self.cur_colour = None # most recent measured colour in RGB
self.hsv = 0
self.num = 0 #keep track of graph plotting
def colour_callback(self, msg):
"""
callback function that receives the most recent colour measurement from the
camera.
"""
self.cur_colour = np.array(msg.data) # [r, g, b]
temp =
[self.cur_colour[0]/255.0,self.cur_colour[1]/255.0,self.cur_colour[2]/255.0]
self.hsv = matplotlib.colors.rgb_to_hsv(temp) # Converts to HSV values
def line_callback(self, msg):
"""
line callback function from lab 3.
"""
self.prev_line_index = self.line_index
self.line_index = int(msg.data)
def wait_for_colour(self):
"""
Loop until a colour is received.
"""
rate = rospy.Rate(100)
while not rospy.is_shutdown() and self.cur_colour is None:
rate.sleep()
def check_colour(self):
"""
returns colour value from HSV measurement
"""
colour_codes = self.colour_codes
cur_colour = self.cur_colour
distance = 1000000 # Set to infinity to see closest colour
j = 4
if abs(self.hsv[1] < 0.060):
# reads black, must be on the line
return 4 #Black line
for i in range(5):
# compares the hue and returns the closest colour
d = abs(self.hsv[0] - colour_codes[i][0])
if d < distance and d < 0.1: #Comparison to previous colour and threshold
j = i
distance = d
return j #returns number of closest colour
def state_predict(self):
rospy.loginfo("predicting state")
"""
state prediction function: update
self.probability with the predicted probability of being at each
state (office)
"""
direction = self.direction
# for our purposes we only go forward since it's a closed loop path
if direction ==1: # go forwards
temp = self.probability.pop(-1)
self.probability.insert(0,temp)
elif direction == -1: # go backwards
temp = self.probability.pop(0)
self.probability.insert(10,temp)
def state_update(self):
rospy.loginfo("updating state")
"""
state update function: update self.probability
with the probability of being at each state
"""
if self.cur_colour is None:
self.wait_for_colour()
prob = []
z_k = self.hsv
# measurement model
for i in range(len(self.probability)):
true_colour = self.colour_map[i]
hsv = self.colour_codes[true_colour]
distance = euclidean_distance(hsv, z_k)
prob.append(1/distance*self.probability[i])
# normalize probability to sum to 1
self.probability = normalize(prob)
def line(self):
diff = abs(self.prev_line_index - self.line_index)
if diff < 250 and 50 < self.line_index < 590:
return True # if line
else:
return False # if not
def find_office(self, office):
"""
main loop for PID control with calls to Bayesian predict and update when at an
office.
when the turtlebot is at the right office, it breaks the loop.
"""
error = 0
prev = 0
twist = self.twist
twist.linear.x = 0.06
self.cmd_pub.publish(twist)
# main loop for PID
while not rospy.is_shutdown():
colour = self.check_colour() #index for which colour
line = self.line()
# PID control setup
kd = 0.0062
kp = 0.006
# Detect if it's line or colour
if line and colour == 4: #If Confident on black line
print("LINE------")
error = 320 - self.line_index
der = error - prev
w = error *kp + der*kd
twist.angular.z = w
self.cmd_pub.publish(twist)
prev = error
elif not line: #Reached an office (Colour) Line index of border of colour
print("-----COLOUR")
# posterior estimate
self.state_update()
# stuff for plotting
x = [2,3,4,5,6,7,8,9,10,11,12]
y = self.probability
num = self.num
plt.bar(x, y)
plt.title("Probability Plot " + str(num))
plt.xlabel("Locations")
plt.ylabel("Probability")
plt.savefig(str(num)+".png")
plt.close()
self.num = self.num +1
# go straight through colour paper until middle of office
twist.linear.x = 0.05
twist.angular.z = 0
self.cmd_pub.publish(twist)
time.sleep(5)
#Prediction
predicted_state_index = self.probability.index(max(self.probability))
predicted_state = states[predicted_state_index]
# check if we're at the right office
if predicted_state == office:
confidence = 0.5 # confidence level for office
# stop if we are at office in the middle and turn
if self.probability[predicted_state_index] > confidence:
twist.linear.x = 0
twist.angular.z = 0
self.cmd_pub.publish(twist) #stops in middle
time.sleep(1)
self.turn_around() #Turn as required
#Go straight until off of colour
twist.linear.x = 0.025
twist.angular.z = 0
self.cmd_pub.publish(twist)
time.sleep(1)
self.state_predict()
break
#Strategy, continue a bit until on line
twist.linear.x = 0.025
twist.angular.z = 0
self.cmd_pub.publish(twist)
time.sleep(1)
prev = 0 #Reset PID
error = 0
self.state_predict()
# update for the next one
elif line and colour <4: # uncertain slow down but continue PID
twist.linear.x = 0.05
# print("line and colour")
error = 320 - self.line_index
der = error - prev
w = error *kp + der*kd
twist.angular.z = w
self.cmd_pub.publish(twist)
prev = error
else: # unknown - probably will never run
print("debug we've hit the edge case")
twist.linear.x = 0
self.cmd_pub.publish(twist)
rate.sleep()
def turn_around(self):
"""
hard coded function so that turtlebot turns 90 degrees to the left, pause 5s,
then return back to original pose
"""
twist = self.twist
twist.linear.x = 0
twist.angular.z = 1.5/4
self.cmd_pub.publish(twist) # rotates
time.sleep(4)
twist.linear.x = 0
twist.angular.z = 0
self.cmd_pub.publish(twist) # look inwards and stops for 2
time.sleep(0.5)
twist.linear.x = 0
twist.angular.z = -1.5/4
self.cmd_pub.publish(twist) # rotate back
time.sleep(4)
twist.linear.x = 0
twist.angular.z = 0
self.cmd_pub.publish(twist) # Make sure it is straight
time.sleep(0.1)
# Additional helper functions
def euclidean_distance(A,B):
x = 0
for i in range(3):
x += (A[i]-B[i])**2
return x**0.5
def normalize(probs):
s = sum(probs)
temp = []
for i in range(len(probs)):
temp.append(probs[i]/s)
return temp # normalized probability
if __name__ == "__main__":
# This is the known map of offices by colour
# 0: red, 1: green, 2: blue, 3: yellow, 4: line
# current map starting at cell #2 and ending at cell #12
red = 0
green = 1
blue = 2
yellow = 3
line = 4
colour_map = [3, 0, 1, 2, 2, 0, 1, 2, 3, 0, 1]
colour_codes = [[0.95, 0.22, 0.74],
[0.33,0.175,0.647],
[0.625,0.34,0.65],
[0.14,0.22,0.75],
[0.83333,0.0175,0.670]]
#Red, Green, Blue, Yellow, Black
states = [2,3,4,5,6,7,8,9,10,11,12]
# initial probability of being at a given office is uniform
# p0 = np.ones_like(colour_map / len(colour_map))
p0 = []
for i in range(len(colour_map)):
p0.append(1.0/float(len(colour_map)))
twist = Twist()
yertle = BayesLoc(p0, colour_codes, colour_map, states, twist) #localizer
rospy.init_node("final_project")
rospy.sleep(0.5)
rate = rospy.Rate(10)
# start before 6
# final office sequence during demonstration
yertle.find_office(9)
yertle.find_office(10)
yertle.find_office(3)
rospy.loginfo("finished!")
plt.show()
# Done
twist.linear.x = 0.00
twist.angular.z = 0.0
yertle.cmd_pub.publish(twist)
time.sleep(2)
# Victory lap spin! SPINNNNNN!!!!!!!!!
twist.linear.x = 0
twist.angular.z = 2
yertle.cmd_pub.publish(twist)
