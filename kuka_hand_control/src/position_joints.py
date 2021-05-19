#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
# import rospkg
import rospy
import time

from std_msgs.msg import Float64MultiArray


q = []
j1_arr = []
j2_arr = []
j3_arr = []
j4_arr = []
j5_arr = []
j6_arr = []
t_arr = []
t_s = time.time()
record_data = True


def callback(data):

    global q, j1_arr, j2_arr, j3_arr, j4_arr, j5_arr, j6_arr, t_s, t_arr, record_data
    if record_data:

        print(data)
        j1_arr.append(data.data[0])
        j2_arr.append(data.data[1])
        j3_arr.append(data.data[2])
        j4_arr.append(data.data[3])
        j5_arr.append(data.data[4])
        j6_arr.append(data.data[5])
        t_arr.append(round((time.time()-t_s)*100, 1))


    else:
        return False


if __name__ == '__main__':

    t_arr.append(0.0)

    rospy.init_node('graph_drawer_j', anonymous=True)
    rospy.Subscriber("joint_position_controller/command", Float64MultiArray, callback)

    t_sleep = 35
    time.sleep(t_sleep)
    record_data = False
    t_arr.pop()

    plt.figure()

    plt.subplot(231)
    plt.plot(t_arr, j1_arr)
    plt.xlabel('time, ms')
    plt.ylabel('J1, rad')
    plt.grid(True)
    plt.title('Joint state 1')


    plt.subplot(232)
    plt.plot(t_arr, j2_arr)
    plt.xlabel('time, ms')
    plt.ylabel('J2, rad')
    plt.grid(True)
    plt.title('Joint state 2')


    plt.subplot(233)
    plt.plot(t_arr, j3_arr)
    plt.xlabel('time, ms')
    plt.ylabel('J3, rad')
    plt.grid(True)
    plt.title('Joint state 3')


    plt.subplot(234)
    plt.plot(t_arr, j4_arr)
    plt.xlabel('time, ms')
    plt.ylabel('J4, rad')
    plt.grid(True)
    plt.title('Joint state 4')


    plt.subplot(235)
    plt.plot(t_arr, j5_arr)
    plt.xlabel('time, ms')
    plt.ylabel('J5, rad')
    plt.grid(True)
    plt.title('Joint state 5')


    plt.subplot(236)
    plt.plot(t_arr, j6_arr)
    plt.xlabel('time, ms')
    plt.ylabel('J6, rad')
    plt.grid(True)
    plt.title('Joint state 6')


    plt.show()
