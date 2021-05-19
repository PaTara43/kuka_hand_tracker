#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
# import rospkg
import rospy
import time

from hand_detector.msg import coordinates
from matplotlib.ticker import (AutoMinorLocator, MultipleLocator)


x_camera = 0.0
y_camera = 0.0
x_arr = []
y_arr = []
t_arr = []
t_s = time.time()
record_data = True


def callback_cam(data):

    global x_camera, y_camera, x_arr, y_arr, t_s, record_data
    if record_data:

        rospy.loginfo("I heard coordinates from camera. \n x: %f, y: %f", data.x_cam, data.y_cam)
        x_camera, y_camera = data.x_cam*100, data.y_cam*100
        x_arr.append(x_camera)
        y_arr.append(y_camera)
        t_arr.append(round((time.time()-t_s)*100, 1))


    else:
        return False


if __name__ == '__main__':

    # x_arr.append(x_camera)
    # y_arr.append(y_camera)
    t_arr.append(0.0)

    rospy.init_node('graph_drawer', anonymous=True)
    rospy.Subscriber("hand_coordinates", coordinates, callback_cam)

    t_sleep = 35
    time.sleep(t_sleep)
    record_data = False
    # x_arr = x_arr[1:-1]
    # y_arr = y_arr[1:-1]
    t_arr.pop()




    fig, (ax1, ax2) = plt.subplots(2, 1)

    fig.suptitle('Error by X, Y')
    ax1.plot(t_arr, x_arr)
    ax1.set(xlabel='time, ms', ylabel='x_error, sm')

    ax2.plot(t_arr, y_arr)
    ax2.set(xlabel='time, ms', ylabel='y_error, sm')
    # Set axis ranges; by default this will put major ticks every 25.
    ax1.set_xlim(0, t_sleep*100)
    ax1.set_ylim(-30, 30)

    # Change major ticks to show every 20.
    ax1.xaxis.set_major_locator(MultipleLocator(t_sleep*20))
    ax1.yaxis.set_major_locator(MultipleLocator(10))

    # Change minor ticks to show every 5. (20/4 = 5)
    ax1.xaxis.set_minor_locator(AutoMinorLocator(10))
    ax1.yaxis.set_minor_locator(AutoMinorLocator(2))

    # Turn grid on for both major and minor ticks and style minor slightly
    # differently.
    ax1.grid(which='major', color='#CCCCCC', linestyle='--')
    ax1.grid(which='minor', color='#CCCCCC', linestyle=':')


    #############################################
    ax2.set_xlim(0, t_sleep*100)
    ax2.set_ylim(-30, 30)

    # Change major ticks to show every 20.
    ax2.xaxis.set_major_locator(MultipleLocator(t_sleep*20))
    ax2.yaxis.set_major_locator(MultipleLocator(10))

    # Change minor ticks to show every 5. (20/4 = 5)
    ax2.xaxis.set_minor_locator(AutoMinorLocator(10))
    ax2.yaxis.set_minor_locator(AutoMinorLocator(2))

    # Turn grid on for both major and minor ticks and style minor slightly
    # differently.
    ax2.grid(which='major', color='#CCCCCC', linestyle='--')
    ax2.grid(which='minor', color='#CCCCCC', linestyle=':')

    plt.show()
