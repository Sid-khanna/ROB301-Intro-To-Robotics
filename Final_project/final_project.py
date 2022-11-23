#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float64MultiArray, UInt32
import numpy as np
import colorsys


class BayesLoc:
    def __init__(self, p0, colour_codes, colour_map):
        self.colour_sub = rospy.Subscriber(
            "mean_img_rgb", Float64MultiArray, self.colour_callback
        )
        self.line_sub = rospy.Subscriber("line_idx", UInt32, self.line_callback)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.num_states = len(p0)
        self.colour_codes = colour_codes
        self.colour_map = colour_map
        self.probability = p0
        self.state_prediction = np.zeros(self.num_states)
        self.guess = 4
        self.line = 0 # value of line
        self.desired = 320
        self.kd = 0.0074
        self.ki = 0.000004
        self.kp = 0.0039
        self.integral = 0
        self.lasterror = 0
        self.twist = Twist()
        self.temp1 = []
        self.temp2 = []
        self.temp3 = []
        
        

        self.cur_colour = None  # most recent measured colour

    def colour_callback(self, msg):
        """
        callback function that receives the most recent colour measurement from the camera.
        """
        self.cur_colour = np.array(msg.data)  # [r, g, b]
        rgb = self.cur_colour
        R = 0
        G = 1
        B = 2
        Y = 3
        L = 4
        guess = None
        
        colour_codes = [
        [232, 79, 128],  # red
        [153, 179, 162],  # green
        [179 , 165, 185],  # blue
        [180, 173, 166],  # yellow
        [170, 159, 162],  # line
        ]
        
        guesses = []
        
        for i in range(len(colour_codes)):
            dist = np.linalg.norm(np.array(self.cur_colour) - np.array(colour_codes[i]))
            guesses.append(dist)
            
        guess = guesses.index(min(guesses))

        # if rgb[0] > 200 and rgb[1] < 100 and rgb[2] > 115:
        #     guess = 'RED'
        # elif rgb[0] < 160 and rgb[1] > 160 and rgb[2] < 170:
        #     guess = 'GREEN'
        # elif rgb[0] > 160 and rgb[1] > 150 and rgb[2] > 175:
        #     guess = 'BLUE'
        # elif rgb[0] > 175 and rgb[0] < 210 and rgb[1] > 140 and rgb[2] > 75:
        #     guess = 'YELLOW'
        # else:
        #     guess = 'LINE'
        
        # if guess == R:
        #     guess = 'RED'
        # elif guess == G:
        #     guess = 'GREEN'
        # elif guess == B:
        #     guess = 'BLUE'
        # elif guess == Y:  
        #     guess = 'YELLOW'
        # elif guess == L:
        #     guess = 'LINE'

        self.guess = guess
        print(guess)
        self.temp1.append(self.cur_colour[0])
        self.temp2.append(self.cur_colour[1])
        self.temp3.append(self.cur_colour[2])
        
        
        
        
        
    def get_colour(self):
        # ''' Guess which colour we are looking at'''
        # R = 0
        # G = 1
        # B = 2
        # Y = 3
        # L = 4
        # sphere = 15
        # guess = None
        
        # if sum(self.cur_colour - self.colour_codes[R])**2 < sphere:
        #     guess = 'RED'
        # elif sum(self.cur_colour - self.colour_codes[G])**2 < sphere:
        #     guess = 'GREEN'
        # elif sum(self.cur_colour - self.colour_codes[B])**2 < sphere:
        #     guess = 'BLUE'
        # elif sum(self.cur_colour - self.colour_codes[Y])**2 < sphere:
        #     guess = 'YELLOW'
        # elif sum(self.cur_colour - self.colour_codes[L])**2 < sphere:
        #     guess = 'LINE'

        # # print("Guess", guess)
        pass

    def line_callback(self, msg):
        """
        TODO: Complete this with your line callback function from lab 3.
        """
        self.line = msg.data
        pass
    
    def control(self):
        error = self.desired - self.line
        self.integral += error
        derivative = error - self.lasterror
        correction = self.kp*error + self.ki*self.integral + self.kd*derivative
        self.twist.linear.x= 0.02
        self.twist.angular.z = correction	
        self.cmd_pub.publish(self.twist)
        self.lasterror= error
        # print(error, correction)
        
    def fwd(self):
        correction = 0
        self.twist.linear.x= 0.02
        self.twist.angular.z = correction
        self.cmd_pub.publish(self.twist)

    def move(self):
        if self.guess == 4:
           self.control()
        else:
            t0 = rospy.Time.now().to_sec()
            while(t0 - rospy.Time.now().to_sec() > -0.1+):
                self.fwd()
                

    def wait_for_colour(self):
        """Loop until a colour is received."""
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and self.cur_colour is None:
            rate.sleep()

    def state_model(self, u):
        for i in range(self.num_states):
            if u == 1:
                self.state_prediction[(i-1)%self.num_states] += self.probability[i]*0.05
                self.state_prediction[(i)%self.num_states] += self.probability[i]*0.10
                self.state_prediction[(i+1)%self.num_states] += self.probability[i]*0.85
            if u == 0:
                self.state_prediction[(i-1)%self.num_states] += self.probability[i]*0.05
                self.state_prediction[(i)%self.num_states] += self.probability[i]*0.90
                self.state_prediction[(i+1)%self.num_states] += self.probability[i]*0.05
            if u == -1:
                self.state_prediction[(i-1)%self.num_states] += self.probability[i]*0.85
                self.state_prediction[(i)%self.num_states] += self.probability[i]*0.10
                self.state_prediction[(i+1)%self.num_states] += self.probability[i]*0.05
        """
        State model: p(x_{k+1} | x_k, u)

        TODO: complete this function
        """

    def measurement_model(self, x):
        """
        Measurement model p(z_k | x_k = colour) - given the pixel intensity,
        what's the probability that of each possible colour z_k being observed?
        """
        
        prob = np.zeros(len(colour_codes))

        """
        TODO: You need to compute the probability of states. You should return a 1x5 np.array
        Hint: find the euclidean distance between the measured RGB values (self.cur_colour)
            and the reference RGB values of each colour (self.ColourCodes).
        """

        return prob

    def state_predict(self):
        rospy.loginfo("predicting state")
        """
        TODO: Complete the state prediction function: update
        self.state_prediction with the predicted probability of being at each
        state (office)
        """

    def state_update(self,col):
        rospy.loginfo("updating state")
        R_I = [3, 4, 7]
        G_I = [1, 5, 9]
        B_I = [2, 6, 10]
        Y_I = [0, 8]
        
        if col == 0:
            for i in R_I:
                self.probability[i] = self.state_prediction[i] * 0.6
            for i in G_I:
                self.probability[i] = self.state_prediction[i] * 0.05
            for i in B_I:
                self.probability[i] = self.state_prediction[i] * 0.05
            for i in Y_I:
                self.probability[i] = self.state_prediction[i] * 0.2
        elif col == 1:
            for i in R_I:
                self.probability[i] = self.state_prediction[i] * 0.05
            for i in G_I:
                self.probability[i] = self.state_prediction[i] * 0.6
            for i in B_I:
                self.probability[i] = self.state_prediction[i] * 0.2
            for i in Y_I:
                self.probability[i] = self.state_prediction[i] * 0.05
        elif col == 2:
            for i in R_I:
                self.probability[i] = self.state_prediction[i] * 0.05
            for i in G_I:
                self.probability[i] = self.state_prediction[i] * 0.2
            for i in B_I:
                self.probability[i] = self.state_prediction[i] * 0.6
            for i in Y_I:
                self.probability[i] = self.state_prediction[i] * 0.05
        elif col == 3:
            for i in R_I:
                self.probability[i] = self.state_prediction[i] * 0.15
            for i in G_I:
                self.probability[i] = self.state_prediction[i] * 0.05
            for i in B_I:
                self.probability[i] = self.state_prediction[i] * 0.05
            for i in Y_I:
                self.probability[i] = self.state_prediction[i] * 0.65

        
    
        
        """
        TODO: Complete the state update function: update self.probabilities
        with the probability of being at each state
        """


if __name__ == "__main__":

    # This is the known map of offices by colour
    # 0: red, 1: green, 2: blue, 3: yellow, 4: line
    # current map starting at cell #2 and ending at cell #12
    colour_map = [3, 1, 2, 0, 0, 1, 2, 0, 3, 1, 2]

    # TODO calibrate these RGB values to recognize when you see a colour
    # NOTE: you may find it easier to compare colour readings using a different
    # colour system, such as HSV (hue, saturation, value). To convert RGB to
    # HSV, use:
    # h, s, v = colorsys.rgb_to_hsv(r / 255.0, g / 255.0, b / 255.0)
    colour_codes = [
        [235, 67, 127],  # red
        [112, 137, 97],  # green
        [176 , 164, 182],  # blue
        [200, 177, 100],  # yellow
        [175, 163, 167],  # line
    ]

    # initial probability of being at a given office is uniform
    p0 = np.ones_like(colour_map) / len(colour_map)

    localizer = BayesLoc(p0, colour_codes, colour_map)

    rospy.init_node("final_project")
    rospy.sleep(0.5)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        """
        TODO: complete this main loop by calling functions from BayesLoc, and
        adding your own high level and low level planning + control logic
        """
        # localizer.get_colour()
        localizer.move()
        rate.sleep()

    rospy.loginfo("finished!")
    print(np.average(localizer.temp1))
    print(np.average(localizer.temp2))
    print(np.average(localizer.temp3))
    rospy.loginfo(localizer.probability)
