#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt32, Float64MultiArray
import matplotlib.pyplot as plt

import numpy as np
import colorsys
import time

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
        self.list = []
        self.D = (420+346)/2
        self.twist = Twist()
        self.integral=0
        self.derivative=0
        self.lasterr =0
        self.cur_colour = None  # most recent measured colour

        self.n = 30
        self.last_n = [None] * self.n
        self.last_col = None

        self.gcount = 0

        self.pause = False
        self.nvisit = 0

        time.sleep(1)

        ##initialize all variables for baysian tracking

        self.XK = [1/11] * 11

        self.map = {'2': 'yellow', '3': 'green', '4': 'blue', '5': 'orange', '6': 'orange', '7': 'green', '8': 'blue', '9': 'orange', '10': 'yellow', '11': 'green', '12': 'blue'}

        # state model
        self.SM = { '-1': [0.85, 0.1, 0.05], '0': [0.05, 0.9, 0.05], '1': [0.05, 0.1, 0.85] }

        # measurement model
        self.MM = {'blue':{'blue':0.6, 'green':0.2, 'orange':0.05, 'yellow':0.05, 'nothing': 0.1}, 
                'green':{'blue':0.2, 'green':0.6, 'orange':0.05, 'yellow':0.05,'nothing': 0.1}, 
        'yellow':{'blue':0.05, 'green':0.05, 'orange':0.15, 'yellow':0.65, 'nothing': 0.1},
        'orange':{'blue':0.05, 'green':0.05, 'orange':0.6, 'yellow':0.2, 'nothing': 0.1}}


    def colour_callback(self, msg):
        """
        callback function that receives the most recent colour measurement from the camera.
        """
        col = ["orange", "green", "blue", "yellow","black"]
        self.cur_colour = np.array(msg.data)  # [r, g, b]
        hsv = []
            
        minv = 1000000
        ind = 0
        L = []
        for i in range(len(col)):
            a = np.array(self.colour_codes[i])
            b = np.array(self.cur_colour)* 100

            # cos_sim = np.dot(a,b)/(np.linalg.norm(a)*np.linalg.norm(b))
            # cos_sim =abs(np.linalg.norm(a)-np.linalg.norm(b))
            cos_sim = np.linalg.norm(a-b)
            if cos_sim < minv:
                minv = cos_sim
                ind = i
            # L.append(cos_sim)   

        temp = col[ind]
        self.last_n.pop(0)
        self.last_n.append(temp)
        # print(np.array(self.cur_colour)* 100)
        # print(temp)


        if ( np.array([temp] * self.n) == np.array(self.last_n) ).all():
            if self.last_col == 'black' and self.last_col != temp or self.last_col != 'black' and 'black' == temp:
                if self.last_col == 'black' and self.last_col != temp :
                    self.nvisit += 1

                    print(self.XK)

                    # run bayesian
                    self.state_predict()
                    plt.bar(self.map.keys(), self.XK)
                    plt.savefig("plot" + str(self.gcount) + 'pred ' + '.png')
                    self.gcount += 1
                    plt.clf()

                    self.state_update(temp)
                    # print(self.XK)
                    st = np.argmax(self.XK) + 2 
                    print(np.argmax(self.XK) + 2)
                    # print("stop bayesian on", temp)
                    print()

                    plt.bar(self.map.keys(), self.XK)
                    plt.savefig("plot" + str(self.gcount) + '.png')
                    self.gcount += 1
                    plt.clf()

                    if int(st) in [2,8,3] and self.nvisit > 2:
                        self.pause = True
                        time.sleep(2)
                        self.pause = False


                self.last_col = temp

    def line_callback(self, msg):
        """
        TODO: Complete this with your line callback function from lab 3.
        """
        speed = 0.05

        self.twist.linear.x= speed
        self.twist.angular.z=0.0
        self.integral, self.derivative = 0,0

        indexes = []

        c = 0
        for i in range(self.n):
            if self.last_n[i] == 'black':
                indexes.append(i)
                c +=1

        indexes = np.array(indexes)
        m = indexes.mean()
        

        if m > self.n/2 or (self.last_col == "black" and c > self.n/1.2):
            # print('line tracking')
            self.actual = msg.data
            kp = 0.002
            ki = 0.0
            kd = 0.01
            rot = 0.1
            self.twist.linear.x= speed

            err = self.D - self.actual
            self.integral += err
            self.derivative = err - self.lasterr

            corr =  1 *(err * kp + self.integral * ki + self.derivative * kd)

            self.twist.angular.z=corr
            self.cmd_pub.publish(self.twist)
            self.lasterr = err
        else:
            self.twist.linear.x= speed
            self.twist.angular.z=0.0
            # self.cmd_pub.publish(self.twist)

        if self.pause == True:
            self.twist.linear.x= 0.0
            self.twist.angular.z=0.0

        self.cmd_pub.publish(self.twist)



    def wait_for_colour(self):
        """Loop until a colour is received."""
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and self.cur_colour is None:
            rate.sleep()

    def state_predict(self):
        # rospy.loginfo("predicting state")
        """
        update
        self.state_prediction with the predicted probability of being at each
        state (office)
        """
        SP = np.zeros(11)
        u = str(1) #robot always moving forward
        for i in range(len(self.XK)):
            j,k = i-1, i+1
            if i == 0:
                j = 10
            elif i == len(self.XK) - 1:
                k = 0
            SP[i] += self.SM[u][2] * self.XK[j] # move forward
            SP[i] += self.SM[u][1] * self.XK[i] # stay
            SP[i] += self.SM[u][0] * self.XK[k] # move backward

        self.XK = SP

    def state_update(self, zk):
        """
        update self.probabilities
        with the probability of being at each state
        """
        SP = np.zeros(11)
        for i in range(len(self.XK)):
            colorx = self.map[str(i + 2)]
            SP[i] = self.XK[i] * self.MM[colorx][zk] # numerator
        norm_factor = sum(SP)
        SP = SP / norm_factor # denominator

        self.XK = SP


if __name__ == "__main__":

    # This is the known map of offices by colour
    # 0: red, 1: green, 2: blue, 3: yellow, 4: line
    # current map starting at cell #2 and ending at cell #12
    colour_map = [3, 0, 1, 2, 2, 0, 1, 2, 3, 0, 1]

    # TODO calibrate these RGB values to recognize when you see a colour
    # NOTE: you may find it easier to compare colour readings using a different
    # colour system, such as HSV (hue, saturation, value). To convert RGB to
    # HSV, use:
    # h, s, v = colorsys.rgb_to_hsv(r / 255.0, g / 255.0, b / 255.0)

    colour_codes = [
    [93, 70, 88],  # red
    [28, 13, 69],  # green
    [78, 11, 69],  # blue
    [7, 11, 63],  # yellow
    [89, 7, 64],  # line
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
        rate.sleep()

    rospy.loginfo("finished!")
    rospy.loginfo(localizer.probability)
