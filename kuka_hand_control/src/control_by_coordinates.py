#!/usr/bin/env python3

import math
import numpy as np
import roboticstoolbox as rtb
import rospkg
import rospy
import time

from hand_detector.msg import coordinates
from roboticstoolbox.robot import *
from sensor_msgs.msg import JointState
from spatialmath import *
from std_msgs.msg import Float64MultiArray

x_camera = 0.0
y_camera = 0.0
Rd = np.array([[0.0,  0.0,  1.0],
            [0.0, -1.0,  0.0],
            [1.0,  0.0,  0.0]]) #desired orientation rotation matrix wtr to base frame


def callback_cam(data):
    global x_camera, y_camera
    x_camera, y_camera = data.x_cam, data.y_cam

def callback_joint_states(data):
    global q, x_camera, y_camera, first_launch, x
    q = np.array(data.position)
    print('q')
    print(q)

    if x_camera==0 or y_camera==0: #in the beginning
        rospy.loginfo('No coordinates from camera')
        return False

    if abs(x_camera)<0.05 and abs(y_camera)<0.05:
        rospy.loginfo('opposite to palm')
        return False


    # FK
    Rc = kuka.fkine(q).data[0][0:3, 0:3] # current rotational matrix from fkine

    # Skew-symmetric
    skew_symm = np.subtract(np.dot(np.linalg.inv(Rd), Rc), np.eye(3))
    vx, vy, vz = skew_symm[2][1], skew_symm[0][2], skew_symm[1][0]
    e = np.array([0.0, -3*x_camera, 3*y_camera, vx, vy, vz]).T


    J = kuka.jacob0(q)
    J_inv = np.linalg.pinv(J)
    dq = np.dot(J_inv, e)
    print('dq')
    print(dq)

    ##look up for max dq, if > 0.03, divide by 0.03 and divide all by the result
    maxvalue = np.amax(abs(dq))
    limit = 0.02
    if maxvalue > limit:
        divfactor = maxvalue/limit
        dq = dq / divfactor

    qnew = q + dq
    print(qnew)
    data_to_send = Float64MultiArray()  # the data to be sent, initialise the array
    data_to_send.data = qnew # assign the array with the value you want to send
    pub.publish(data_to_send)


robot = DHRobot(
[
    RevoluteDH(d=-0.4, a=0.025, alpha=math.pi/2),
    RevoluteDH(a=0.455),
    RevoluteDH(offset=-math.pi/2, a=0.035, alpha=math.pi/2),
    RevoluteDH(d=-0.4186, alpha=-math.pi/2),
    RevoluteDH(alpha=math.pi/2),
    RevoluteDH(d=-0.08, alpha=math.pi)
], name="KUKAKR6", base=SE3.RPY([0.0, 0.0, math.pi], order='xyz')
        )

class KUKAKR6(DHRobot):

    def __init__(self):
        super().__init__(
                [
                    RevoluteDH(d=-0.4, a=0.025, alpha=math.pi/2),
                    RevoluteDH(a=0.455),
                    RevoluteDH(offset=-math.pi/2, a=0.035, alpha=math.pi/2),
                    RevoluteDH(d=-0.4186, alpha=-math.pi/2),
                    RevoluteDH(alpha=math.pi/2),
                    RevoluteDH(d=-0.08, alpha=math.pi)
                ], name="KUKAKR6", base=SE3.RPY([0.0, 0.0, math.pi], order='xyz')
                        )

if __name__ == '__main__':

    kuka = KUKAKR6()
    print(kuka)
    first_launch = True

    rospy.init_node('control_generator', anonymous=True)
    pub = rospy.Publisher('/joint_position_controller/command', Float64MultiArray, queue_size=10)
    rospy.Subscriber("hand_coordinates", coordinates, callback_cam)
    rospy.Subscriber("joint_states", JointState, callback_joint_states)
    rospy.spin()
