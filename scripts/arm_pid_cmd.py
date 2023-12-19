#!/usr/bin/python3

import rospy
from std_msgs.msg import Float32

import matplotlib.pyplot as plt

import time
import math

plot_size = 6000

x = [i for i in range(plot_size)]
y = [0 for i in range(plot_size)]
y_ref_list = [0 for i in range(plot_size)]



i = 0
model = True
t = 0



def plot(x, y, y_ref):
    plt.clf()
    plt.plot(x, y)
    plt.plot(x, y_ref)
    plt.draw()
    plt.pause(0.001)



if __name__ == "__main__":
    rospy.init_node("arm_pid_plot", anonymous=True)
    topic_name = "/Crp450/Arm1_joint"

    
    pub = rospy.Publisher(topic_name+"/vel_cmd", Float32, queue_size=10) 
    time.sleep(10)
    
    t = 0
    y_ref = 0
    floor = 1.0
    cell = 3.0
    while(1):  
        time.sleep(0.01)
        t += 1
        
        
        # y_ref = 3*math.sin(t/50)
        # y_ref = 0
        
        if t > 300:
            model = ~model
            t = 0
        
        if model == True:
            y_ref = cell
        else:
            y_ref = floor
        
        
        print(model, y_ref)
        pub.publish(y_ref)
        

    rospy.spin()
    