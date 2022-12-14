#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float64MultiArray, UInt32
import numpy as np
import colorsys
import matplotlib.pyplot as plt


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
       
        self.prev_colour = 100
        self.guess = 4
        self.colour = 4
        self.line_idx = 0 # value of line
        #Apriori
        self.state_prediction = np.zeros(self.num_states)
        #posteriori
        self.probability = (1/self.num_states)*np.ones(self.num_states)
        self.u = 1
        #predict
        self.add = 0
        self.conf = 0
        self.counter = 0
        self.goal = [4,3,6]
        self.update = np.zeros((self.num_states,self.num_states))
        #control
        self.hsv = (0, 0, 0)
        self.desired = 320
        self.kd = 0.009
        self.ki = 0.000004
        self.kp = 0.0039
        self.integral = 0
        self.lasterror = 0
        self.twist = Twist()
        self.temp1 = []
        self.temp2 = []
        self.temp3 = []
        self.prev_line_idx = 0
        self.on_line = False
        

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
        
        hsv_colour_codes = [
            [0.9539, 0.7082, 0.8960], #red
            [0.4181,0.1585,0.6905], #g
            [0.775,0.0656,0.6355], #b
            [0.1327,0.1504,0.6372], #y
            [0.3997,0.0257,0.587]  # line
        ]
        
        guesses = []
        hsv_guesses = []
        
        for i in range(len(colour_codes)):
            dist = np.linalg.norm(np.array(self.cur_colour) - np.array(colour_codes[i]))
            guesses.append(dist)
            
        guess = guesses.index(min(guesses))
        
        
        self.hsv = colorsys.rgb_to_hsv(self.cur_colour[0] / 255.0, self.cur_colour[1] / 255.0, self.cur_colour[2] / 255.0)
        distance = 100000
        j = 4
        
        # if abs(self.hsv[1]<0.099):
        #     hsv_guess = 4
        # else:
        for i in range(len(hsv_colour_codes)):
        #         d = abs(self.hsv[0] - hsv_colour_codes[i][0])
        #         if d < distance and d<0.1:
        #             j = i
        #             distance = d
                # hsv_guess = j
            dist = np.linalg.norm(np.array(self.hsv) - np.array(hsv_colour_codes[i]))
            hsv_guesses.append(dist)
        hsv_guess = hsv_guesses.index(min(hsv_guesses))
        self.temp1.append(self.hsv[0])
        self.temp2.append(self.hsv[1])
        self.temp3.append(self.hsv[2])
        self.guess = hsv_guess
        
        
        #hsv_guess = hsv_guesses.index(min(hsv_guesses))
        # print(self.hsv)
        # print('hsv:',hsv_guess)
        
        
    def get_colour(self):
        self.colour = self.guess
        pass

    def line_callback(self, msg):
        """
        TODO: Complete this with your line callback function from lab 3.
        """
        self.prev_line_idx = self.line_idx
        self.line_idx = msg.data
        pass
    
    def control(self, speed):
        error = self.desired - self.line_idx
        self.integral += error
        derivative = error - self.lasterror
        correction = self.kp*error + self.ki*self.integral + self.kd*derivative
        self.twist.linear.x= speed
        self.twist.angular.z = correction	
        self.cmd_pub.publish(self.twist)
        self.lasterror= error
        # print(error, correction)
        
    def line(self):
        diff = abs(self.prev_line_idx - self.line_idx)
        if diff < 250 and 50 < self.line_idx < 590:
            return True
        else:
            return False
                

    def wait_for_colour(self):
        """Loop until a colour is received."""
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and self.cur_colour is None:
            rate.sleep()

    def state_predict(self, u):
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

    '''def state_predict(self):
        rospy.loginfo("predicting state")
        """
        TODO: Complete the state prediction function: update
        self.state_prediction with the predicted probability of being at each
        state (office)
        """'''
    def normalize(self, l):
        norm = 0
        for i in range(len(l)):
            norm += l[i]
        for i in range (len(l)):
            l[i] = l[i]/norm
        return l

    def state_update(self,col):
        # rospy.loginfo("updating state")
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
                
                
        self.probability = self.normalize(self.probability)
        print(f'Probability {self.probability}')
        best_guess = max(self.probability)
        best_location = np.where(best_guess == self.probability)
        return best_location, best_guess
    
    def fwd(self):
        correction = 0
        self.twist.linear.x= 0.05
        self.twist.angular.z = correction
        self.cmd_pub.publish(self.twist)
    
    def stop(self):
        correction = 0
        self.twist.linear.x= 0
        self.twist.angular.z = correction
        self.cmd_pub.publish(self.twist)
    
    def run(self):
        self.move()
        self.state_predict(self.u)
        self.add, self.conf = self.state_update(self.guess)
        self.update[self.counter] = np.array(self.probability)
        print(self.add, self.guess, self.conf)
        self.counter += 1
        if self.add in self.goal and self.conf >0.5:
            #do the whole turning thing
            pass
                
    def move(self):
        self.get_colour()
        line_t = self.line()
        if self.colour == 4 and line_t:
            self.control(0.08)
        elif not line_t:
            self.fwd()
            rospy.sleep(0.5)
            self.get_colour()
            if self.colour != self.prev_colour:
                self.prev_colour = self.colour
                self.state_predict(self.u)
                self.add, self.conf = self.state_update(self.guess)
                # self.update[self.counter] = np.array(self.probability)
                print(f'addr: {self.add}, guess: {self.colour}, conf: {self.conf}')
            # self.counter += 1
            # if self.add in self.goal and self.conf >0.5:
            #     self.fwd()
            #     rospy.sleep(4.5)
            #     self.turn()
            #     self.fwd()
            #     rospy.sleep(4.5)
            # else:
            self.fwd()
        elif line_t and self.colour < 4:
            self.colour = 4
            self.control(0.05)
        else:
            self.stop()
        #     rospy.sleep(1)
        #     self.fwd()
            



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

    hsv_colour_codes = [
        [0.95, 0.22, 0.74], #red
        [0.33,0.175,0.647], #g
        [0.625,0.34,0.65], #b
        [0.14,0.22,0.75], #y
        [0.83333,0.0175,0.670]  # line
    ]
    
    # initial probability of being at a given office is uniform
    p0 = np.ones_like(colour_map) / len(colour_map)

    localizer = BayesLoc(p0, colour_codes, colour_map)

    rospy.init_node("final_project")
    rospy.sleep(0.5)
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        """
        TODO: complete this main loop by calling functions from BayesLoc, and
        adding your own high level and low level planning + control logic
        """
        # localizer.get_colour()
        localizer.move()
        rate.sleep()

    rospy.loginfo("finished!")
    plt.plot(localizer.probability)
    # print(np.average(localizer.temp1))
    # print(np.average(localizer.temp2))
    # print(np.average(localizer.temp3))
    rospy.loginfo(localizer.probability)
