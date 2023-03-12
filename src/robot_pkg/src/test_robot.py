#!/usr/bin/env python
# -*- coding:utf-8 -*-

# import sys
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
# import cv2 as cv
# # make sure to use import rospy in the future
# sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages') 
import rospy
# import cv2 as cv
from roboticstoolbox import DHRobot,RevoluteDH,PrismaticDH,RevoluteMDH,PrismaticMDH
import roboticstoolbox as rtb
from math import pi
import math
import numpy as np





######
#   i    | alpha_i-1  |
#
#
#
#
#
#
#
######

class TriRobot(DHRobot):

    def __init__(self, l_b,l_s,l_h):
        deg = pi/180

        self.l_b = l_b
        self.l_s = l_s
        self.l_h = l_h
        # L0 = RevoluteDH(
        #     d=0,
        #     a=0,
        #     alpha=pi/2
            
        # )
        L1 = RevoluteMDH(
            d=0,
            a=0,
            alpha=0
            
        )


        L2 = RevoluteMDH(
            d=0,
            a=0,
            alpha=pi/2

        )

        L3 = RevoluteMDH(
            d=self.l_b,
            a=0,
            alpha=pi/2
        )

        L4 = RevoluteMDH(
            d=0,
            a=0,
            alpha=-pi/2
        )

        L5 = RevoluteMDH(
            d=self.l_s,
            a=0,
            alpha=pi/2
        )

        L6 = RevoluteMDH(
            d=0,
            a=0,
            alpha=pi/2
        )
        
        L7 = RevoluteMDH(
            d=0,
            a=0,
            alpha=pi/2
        )

        super().__init__(
            [L1,L2,L3,L4,L5,L6,L7], 
            name="UPPERLIMB",
        )

        self._MYCONFIG = np.array([1,2,3,4,5,6,7])
        self.qr = np.array([0,0,0,0,0,0])
        self.qz = np.array([1,0,0,0,0,0])
        
        self.addconfiguration("qz", self.qr)
        self.addconfiguration("qr", self.qz) 



if __name__ == '__main__':
    # robot = UPPERLIMB(l_b=0.3, l_s=0.4, l_h=0.05)

    # q0 = np.array([0,   -pi/2,pi/2,pi,  0,   pi/2,0])

    # q1 = np.array([pi/2,-pi/2,pi/2,pi,  0,   pi/2,0])

    # q2 = np.array([0,   0,   pi/2,pi,  0,   pi/2,0])

    # q3 = np.array([0,   -pi/2,pi,pi,  0,   pi/2,0])


    # q4 = np.array([0,   -pi/2, pi/2, pi/2,  0,   pi/2,0])
    # # print(robot)      
    # qt = rtb.jtraj(q0,q1,100)
    # robot.plot(qt.q)

    robot = TriRobot(l_b=0.5,l_s=0.4,l_h=0.05)
    q0 = np.array([0,    pi/2,  -pi/2,  0,    pi/2,  pi/2,  0])
    q1 = np.array([pi/2, pi/2,  -pi/2,  0,    pi/2,  pi/2,  0])
    q2 = np.array([0,    pi,    -pi/2,  0,    pi/2,  pi/2,  0])
    q3 = np.array([0,    pi/2,  0,      0,    pi/2,  pi/2,  0])
    q4 = np.array([0,    pi/2,  -pi/2,  pi/2, pi/2,  pi/2,  0])
    q5 = np.array([0,    pi/2,  -pi/2,  0,    pi,    pi/2,  0])
    q6 = np.array([0,    pi/2,  -pi/2,  0,    pi/2,  pi,    0])
    q7 = np.array([0,    pi/2,  -pi/2,  0,    pi/2,  pi/2,  pi/2])
    
    T11 = np.array([[1, 0, 0, 0.05],
                    [0, 1, 0, 0   ],
                    [0, 0, 1, 0   ],
                    [0, 0, 0, 1]])
    q00 = np.array([0,    pi/2,  -pi/2,  0,    pi/2,  pi/2+pi/6,  0])
    q07 = np.array([0,    pi/2,  -pi/2,  0,    pi/2,  pi/2+pi/6,  pi/2])
    q0007 = np.array([0,    pi/2,  -pi/2,  0,    pi/2,  pi/2,  pi/2])
    qt = rtb.jtraj(q00,q07,100)
    robot.plot(qt.q)

    fk1 = robot.fkine(q0007)*T11
    fk2 = robot.fkine(q07)*T11
    Ja1 = robot.jacob0(q00)
    Ja2 = robot.jacob0(q07)
    print(fk1)
    print(fk2)
    # print(Ja1)
    # print('\n')
    # print(Ja2) 