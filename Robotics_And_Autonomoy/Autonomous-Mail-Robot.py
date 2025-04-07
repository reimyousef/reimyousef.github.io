"""
Autonomous Mail Robot - ROS Python Node
-----------------------------------------
This ROS-based Python script enables an autonomous mobile robot to deliver to specific virtual "offices"
by following a black line path and detecting colored office markers using camera-based color recognition.

The robot employs:
- **Bayesian localization** to update belief of current position based on color measurements.
- **Line following with PID control** to navigate smoothly along a pre-defined path.
- **Color segmentation** and matching to identify office markers.
- **Motion interruption and delivery logic** when the robot reaches its assigned target office(s).


Course: ROB301, University of Toronto

"""

#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt32, Float64MultiArray
import numpy as np
import colorsys
import matplotlib.pyplot as plt 


class BayesLoc:
    def __init__(self, p0, colour_codes, colour_map, office_input):
        self.colour_sub = rospy.Subscriber("mean_img_rgb", Float64MultiArray, self.colour_callback)
        self.line_sub = rospy.Subscriber("line_idx", UInt32, self.line_callback)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.num_states = len(p0)
        self.colour_codes = colour_codes
        self.colour_map = colour_map
        self.probability = p0
        self.state_prediction = np.zeros(self.num_states)
        self.actual = None #what camera sees
        self.cur_colour = None  
        self.check_office = True  #added this
        self.most_recent_office = None #added this
        self.belief_list = [] # added this 
        self.office_input = office_input
        self.local_count = 0

    def colour_callback(self, msg):
        self.cur_colour = np.array(msg.data)  # [r, g, b]
      #  print(self.cur_colour)

    def line_callback(self, msg):
        self.actual=msg.data
        return

    def wait_for_colour(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and self.cur_colour is None:
            rate.sleep()

    def state_model(self, u):

        # given matrix 
        state_model_mat = np.array([[0.05, 0.05, 0.85]])
        return state_model_mat[0][u]
        

    def measurement_model(self, x):
        """
        Measurement model p(z_k | x_k = colour) - given the pixel intensity,
        what's the probability that of each possible colour z_k being observed?
        """
        if self.cur_colour is None:
            self.wait_for_colour()

        prob = np.zeros(len(self.colour_codes)) 
        # GIVEN that the current state is some colour, what is the probability that we measure each colour at each index of the array
        dist = np.zeros(len(self.colour_codes))
        for i in range(0, 5):
            distance = np.linalg.norm(self.cur_colour - self.colour_codes[i])
            print("distance array measuring", distance)
            dist[i] = distance

        # Get the min distance from the array and the index, that will be the measurement we assume 
        measurement = np.argmin(dist)
        print(measurement, "measured this")
        
        mes_table = np.array([[0.6, 0.2, 0.05, 0.05],[0.2, 0.6, 0.05, 0.05], [0.05, 0.05, 0.65, 0.2], [0.05, 0.05, 0.15, 0.6], [0.1,0.1,0.1,0.1]])
        prob = mes_table[measurement, :]
        return prob

    def state_predict(self):
        rospy.loginfo("predicting state")
        n = len(self.probability)
        initial_belief = self.probability[:]
        updated_belief = np.zeros(n)
        for x in range(n):
            updated_belief[(x-1)%n] += 0.05 * initial_belief[(x) % n]
            updated_belief[x%n] += 0.1 * initial_belief[x % n] 
            updated_belief[(x + 1)%n] += 0.85 * initial_belief[(x) % n]
        self.state_prediction = updated_belief
        

    def state_update(self):
        rospy.loginfo("updating state")
        """
        TODO: Complete the state update function: update self.probabilities
        with the probability of being at each state
        """
        self.state_predict()
        new_belief = self.state_prediction
        prob=self.measurement_model(1) #1 by 4
        for x in range(len(self.probability)):
            new_belief[x] *= prob[[colour_map[x]]]
        # Normalize
        total_belief = np.sum(new_belief)
        if total_belief > 0:
            new_belief /= total_belief  
        else:
            raise ValueError("Belief sum is zero, normalization failed.")
        self.probability = new_belief #probability 

    def define_move(self, x,y,z, twist, rate):
        twist.linear.x=x
        twist.linear.y=y
        twist.angular.z=z
        self.cmd_pub.publish(twist)
    
    def follow_the_line_pid(self):
        desired=320
        integral=0
        derivative=0
        lasterror=0
        twist=Twist()
        rate=rospy.Rate(10)
        ki=0.000003
        kp=0.004
        kd=0.00005
        integral=0
        #removed this from here:  check_office = True #this means we are on the white line
        while not rospy.is_shutdown(): #this loop runs 100 times per second.
            if self.actual is not None:
             # print(self.cur_colour)
            
                if self.check_office==True: #looking for office (updates self.check_office)
                    current_color_we_see=self.cur_colour #1 by 3
                    last_seen_office_colour=current_color_we_see
                    
                    for i in range(0, 4):
                        
                        difference=[0]*3 
                        cur_colour=colour_codes[i] #1 by 3
                        
                        difference[0] = current_color_we_see[0]-cur_colour[0]
                        difference[1] = current_color_we_see[1]-cur_colour[1]
                        difference[2] = current_color_we_see[2]-cur_colour[2]
                        
                        if abs(difference[0])<20 and abs(difference[1])<20 and abs(difference[2])<20:
                            self.check_office = False # will stop checking office
                            self.state_update()
                            print(self.probability, "self.prob!!!!!!!!!!")
                            self.belief_list
                            print("IS IT NORMAL", sum(self.probability))
                            print(self.cur_colour, "colour i see")
                            office_state = np.argmax(self.probability)+2
                            print("should be at office", office_state)
                            self.belief_list.append(self.probability)
                            print("APPENDEDDD", self.belief_list)
                            self.most_recent_office = i 
                            print("COLOR", i, "DETECTED!!!!!!!!")

                            # counting how many offices we have passed 
                            self.local_count += 1
                            
                            # checking if our current office is the office we are stopping at. it will only do this check if we have gone around once 
                            if office_state == self.office_input[0] and self.local_count > 11:
                                rospy.sleep(5) # STOPS ROVER
                                # we already stopped here 
                                self.office_input.pop(0)

                
                if self.check_office == False: #we are at an office.
                    current_color_we_see=self.cur_colour
                    
                    twist.linear.x = 0.05
                    twist.angular.z = 0 
                    self.cmd_pub.publish(twist)
                        # check if we can have reached a line.
                    difference=[0]*3 
                    cur_colour=colour_codes[4] #line colors 
                    
                    difference[0] = current_color_we_see[0]-cur_colour[0]
                    difference[1] = current_color_we_see[1]-cur_colour[1]
                    difference[2] = current_color_we_see[2]-cur_colour[2]
                    '''
                    difference[0] = last_seen_office_colour[0]-current_color_we_see[0]
                    difference[1] = last_seen_office_colour[1]-current_color_we_see[1]
                    difference[2] = last_seen_office_colour[2]-current_color_we_see[2]
                    '''
                    if abs(difference[0])<20 and abs(difference[1])<20 and abs(difference[2])<20:
                       
                        self.check_office = True # will start checking for offices again
                
                elif self.check_office == True: # we are not at an office, so follow the white line
                    
                    error=desired-self.actual
                    integral=integral+error
                    derivative=error-lasterror
                    self.define_move(0.05, 0,  (kp*error)+(ki*integral)+kd*derivative, twist, rate)#(kp*error)+(ki*integral)+kd*derivative, twist, rate) 
                    lasterror=error
                rospy.sleep(0.005)
            

if __name__ == "__main__":

    # This is the known map of offices by colour
    # 0: orange, 1: purple, 2: red, 3: brown, 4: line
    # current map starting at cell #2 and ending at cell #12
    colour_map = [2, 1, 0, 0, 0, 1, 0, 0, 2, 1, 0]

    # TODO calibrate these RGB values to recognize when you see a colour
    # NOTE: you may find it easier to compare colour readings using a different
    # colour system, such as HSV (hue, saturation, value). To convert RGB to
    # HSV, use:
    # h, s, v = colorsys.rgb_to_hsv(r / 255.0, g / 255.0, b / 255.0)
    colour_codes = [
        # [242, 74, 130], # orange 
        # [248, 144, 92], # purpple
        # [215, 125, 108],  #red
        # [214, 124, 127], #brown
        # given
        [215, 150, 130],  # orange
        [187,151,204],# blue
        [232, 78, 125],  # red
        [204, 145, 125], # brown [208, 123, 110]
        [164, 150, 160] #line
          # line,
    ]

    # initial probability of being at a given office is uniform
    p0 = np.ones_like(colour_map) / len(colour_map) #array of 1/11 for all offices
    stop_office = [2, 11, 6]


    localizer = BayesLoc(p0, colour_codes, colour_map)

    rospy.init_node("final_project")
    rospy.sleep(0.5)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown(): #runs 10 times per second
        """
        TODO: complete this main loop by calling functions from BayesLoc, and
        adding your own high level and low level planning + control logic
        """
        localizer.follow_the_line_pid()
        rate.sleep()

    rospy.loginfo("finished!")
    rospy.loginfo(localizer.probability)
    
    print("HERE IS OUR Belief list", localizer.belief_list)
    for belief in localizer.belief_list:
        print(belief, "BEKLLL")
        plt.figure(figsize=(6, 4))  # Create a new figure for each graph
        x = np.arange(len(belief))  # X positions
        plt.bar(x, belief, color='blue', alpha=0.7)  # Bar graph
        plt.title(f"Bar Graph")
        plt.xlabel("Index")
        plt.ylabel("Values")
        plt.xticks(x)  # Set x-ticks to match the indices
        plt.grid(axis='y', linestyle='--', alpha=0.6)
        plt.show()

   