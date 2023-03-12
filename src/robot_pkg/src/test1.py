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
            alpha=-pi/2
        )

        L4 = RevoluteMDH(
            d=0,
            a=0,
            alpha=-pi/2
        )

        L5 = RevoluteMDH(
            d=self.l_s,
            a=0,
            alpha=-pi/2
        )

        L6 = RevoluteMDH(
            d=0,
            a=0,
            alpha=pi/2
        )
        
        L7 = RevoluteMDH(
            d=0,
            a=self.l_h,
            alpha=-pi/2
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




class UPPERLIMB(DHRobot):


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
        L0 = RevoluteDH(
            d=0,
            a=0,
            alpha=pi/2
            
        )

        L1 = RevoluteDH(
            d=0,
            a=0,
            alpha=pi/2

        )

        L2 = RevoluteDH(
            d=self.l_b,
            a=0,
            alpha=-pi/2
        )

        L3 = RevoluteDH(
            d=0,
            a=0,
            alpha=-pi/2
        )

        L4 = RevoluteDH(
            d=self.l_s,
            a=0,
            alpha=pi/2
        )

        L5 = RevoluteDH(
            d=0,
            a=0,
            alpha=-pi/2
        )

        L6 = RevoluteDH(
            d=0,
            a=self.l_h,
            alpha=0
        )

        super().__init__(
            [L0,L1,L2,L3,L4,L5,L6], 
            name="UPPERLIMB",
        )

        self._MYCONFIG = np.array([1,2,3,4,5,6,7])
        self.qr = np.array([0,0,0,0,0,0,0])
        self.qz = np.array([1,1,1,1,1,1,1])
        self.addconfiguration("qz", self.qr)
        self.addconfiguration("qr", self.qz) 

    def MYCONFIG(self):
        return self._MYCONFIG

    def tranCoordinate(self, p_w):
        """
        输入关节点的世界坐标系位置，转换到基坐标系

        为了方便起见,现对六个点编号：
        1.左肩上的特殊点为A
        2.右肩上的特殊点为B
        3.右肘上的特殊点为C
        4.右腕上的特殊点为D
        5.右手手掌心的特殊点为点E
        6.右手食指根部关节上的特殊点为F

        基坐标系的三个轴按如下确定：
        u轴(x轴):由左肩指向右肩
        v轴(z轴):竖直向上
        w轴(y轴):根据右手坐标系法则确定

        param p_w: 六个特殊点的世界坐标(point_world,基于世界坐标系)
        type p_w: ndarray(3,6)
        """
        A_w = p_w[:,0].reshape(3,1)
        B_w = p_w[:,1].reshape(3,1)
        C_w = p_w[:,2].reshape(3,1)
        D_w = p_w[:,3].reshape(3,1)
        E_w = p_w[:,4].reshape(3,1)
        F_w = p_w[:,5].reshape(3,1)

        ##做平移变换，以右肩点为原点，以下为各点经过平移变换后的点
        A_w = A_w - B_w
        B_w = B_w - B_w
        C_w = C_w - B_w
        D_w = D_w - B_w
        E_w = E_w - B_w
        F_w = F_w - B_w

        ##做旋转变换，先做出人体上肢坐标系各轴在世界坐标系下的坐标表示（已归一化）：
        u_x = -A_w[0,0]
        u_y = -A_w[1,0]
        u_z = 0
        u = np.array([[u_x, u_y, u_z]])
        u = u/np.linalg.norm(u) #以左肩指向右肩为u轴,由于肩部两端不一定在同一水平面上，此处设定u在z方向上一定为0

        w = np.array([[0, 0, 1]]) #设定w轴一定竖直向上

        v = np.cross(w,u) #用右手坐标系法则确定v轴方向

        coor_base = np.array([u,v,w]).squeeze() #上肢坐标系在相机坐标系下的表示

        A_b = np.dot(coor_base, A_w)
        B_b = np.dot(coor_base, B_w)
        C_b = np.dot(coor_base, C_w)
        D_b = np.dot(coor_base, D_w)
        E_b = np.dot(coor_base, E_w)
        F_b = np.dot(coor_base, F_w)

        p_b = np.hstack([A_b,B_b,C_b,D_b,E_b,F_b])

        return p_b

    def angleCul(self, p_b):
        """
        输入关节点的位置，获取关节角度信息
        
        为了方便起见,现对六个点编号：
        1.左肩上的特殊点为A
        2.右肩上的特殊点为B
        3.右肘上的特殊点为C
        4.右腕上的特殊点为D
        5.右手手掌心的特殊点为点E
        6.右手食指根部关节上的特殊点为F

        param p_b: 六个特殊点的世界坐标(point_base,基于基坐标系)
        type p_b: ndarray(3,6)
        """

        A = p_b[:,0].reshape(3,1)
        B = p_b[:,1].reshape(3,1)
        C = p_b[:,2].reshape(3,1)
        D = p_b[:,3].reshape(3,1)
        E = p_b[:,4].reshape(3,1)
        F = p_b[:,5].reshape(3,1)


        ##以下角度都为弧度制
        q1 = math.atan(C[1]/C[0])
        q2 = -math.acos(-C[2]/np.linalg.norm(C))

        q4 = math.acos(np.dot((B-C),(D-C))/np.linalg.norm(B-C)*np.linalg.norm(D-C))
        
        #q3比较复杂，此处做一些拆分：fz1表示分子上第一块东西，fz2表示分子第二块东西，fm1表示分母第一块东西
        fz1_1 = self.l_s*math.cos(q2)*math.cos(q4)
        fz2_1 = self.l_b*math.cos(q2)
        fz3_1 = D[2]
        fm1_1 = math.sin(q2)*math.sin(q4)
        q3 = math.acos((fz1_1-fz2_1-fz3_1)/fm1_1)

        #q5,以下表示平面的向量为该平面的法向量
        #q5,以下表示平面的向量为该平面的法向量
        




        return 0


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

    # robot = TriRobot(l_b=0.5,l_s=0.4,l_h=0.05)
    # q0 = np.array([0,-pi/2,-pi/2,pi,pi,pi/2,0])
    # q1 = np.array([0,-pi/2,-pi/2,pi,pi,pi,0])
    # qt = rtb.jtraj(q0,q1,100)
    # robot.plot(qt.q)
    B = np.array([-2,0,0])
    C = np.array([0,0,0])
    D = np.array([1,1.732,0])
    fz11 = np.dot((C-B),(D-C))
    fm11 = np.linalg.norm(B-C)*np.linalg.norm(D-C)
    q4 = math.acos((np.dot((C-B),(D-C)))/(np.linalg.norm(B-C)*np.linalg.norm(D-C)))
    f = fz11/fm11
    print(q4*180/pi)