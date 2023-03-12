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
from math import pi, cos, sin
import math
import numpy as np


class UPPERLIMB(DHRobot):


    def __init__(self, p_w):
        deg = pi/180

        self.l_b = np.linalg.norm(p_w[:,1]-p_w[:,2])
        self.l_s = np.linalg.norm(p_w[:,2]-p_w[:,3])
        self.l_h = np.linalg.norm(p_w[:,3]-p_w[:,4])

        self.L1 = RevoluteMDH(
            d=0,
            a=0,
            alpha=0
            
        )


        self.L2 = RevoluteMDH(
            d=0,
            a=0,
            alpha=pi/2

        )

        self.L3 = RevoluteMDH(
            d=self.l_b,
            a=0,
            alpha=pi/2
        )

        self.L4 = RevoluteMDH(
            d=0,
            a=0,
            alpha=-pi/2
        )

        self.L5 = RevoluteMDH(
            d=self.l_s,
            a=0,
            alpha=pi/2
        )

        self.L6 = RevoluteMDH(
            d=0,
            a=0,
            alpha=pi/2
        )
        
        self.L7 = RevoluteMDH(
            d=0,
            a=0,
            alpha=pi/2
        )

        super().__init__(
            [self.L1,self.L2,self.L3,self.L4,self.L5,self.L6,self.L7], 
            name="UPPERLIMB",
        )

        self._MYCONFIG = np.array([1,2,3,4,5,6,7])
        self.qr = np.array([0,0,0,0,0,0])
        self.qz = np.array([1,0,0,0,0,0])
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
        type p_w: ndarray(3,7)
        """
        A_w = p_w[:,0].reshape(3,1)
        B_w = p_w[:,1].reshape(3,1)
        C_w = p_w[:,2].reshape(3,1)
        D1_w = p_w[:,3].reshape(3,1)
        D2_w = p_w[:,4].reshape(3,1)
        E_w = p_w[:,5].reshape(3,1)
        F_w = p_w[:,6].reshape(3,1)

        ##做平移变换，以右肩点为原点，以下为各点经过平移变换后的点
        A_w = A_w - B_w
        B_w = B_w - B_w
        C_w = C_w - B_w
        D1_w = D1_w - B_w
        D2_w = D2_w - B_w
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
        D1_b = np.dot(coor_base, D1_w)
        D2_b = np.dot(coor_base, D2_w)
        E_b = np.dot(coor_base, E_w)
        F_b = np.dot(coor_base, F_w)

        p_b = np.hstack([A_b,B_b,C_b,D1_b,D2_b,E_b,F_b])

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
        type p_b: ndarray(3,7)
        """

        A = p_b[:,0].reshape(3,1)
        B = p_b[:,1].reshape(3,1)
        C = p_b[:,2].reshape(3,1)
        D1 = p_b[:,3].reshape(3,1)
        D2 = p_b[:,4].reshape(3,1) #D2为手腕上靠近大拇指一侧的位置
        D = (D1+D2)/2
        E = p_b[:,5].reshape(3,1)
        F = p_b[:,6].reshape(3,1)


        ##以下角度都为弧度制
        q1 = math.atan(C[1]/C[0])   #范围在-pi/2到pi/2之间

        q2 = math.asin(C[2]/np.linalg.norm(C)) + pi/2 #asin的范围在-pi/2到pi/2之间，但是此处需要0到pi之间，因此另外加一个pi/2

        q4 = math.acos((np.dot((C-B),(D-C)))/(np.linalg.norm(B-C)*np.linalg.norm(D-C))) #acos的范围在0到pi之间
        
        #q3比较复杂，此处做一些拆分：fz1表示分子上第一块东西，fz2表示分子第二块东西，fm1表示分母第一块东西
        fz1_1 = self.l_s*cos(q2)*cos(q4)
        fz2_1 = self.l_b*cos(q2)
        fz3_1 = D[2]
        fm1_1 = sin(q2)*sin(q4)
        q3 = math.acos(-(fz1_1+fz2_1+fz3_1)/fm1_1)

        #q5,以下表示平面的向量为该平面的法向量
        l_d1d2 = D2-D1
        bcd = np.cross(C-B,D-C)
        q5 = math.acos(np.dot(bcd, l_d1d2) / (np.linalg.norm(bcd) * np.linalg.norm(l_d1d2)))

        #q6,以下表示平面的向量为该平面的法向量
        l_cd = D-C
        l_de = E-D
        l_ef = F-E
        dfe = np.cross(l_de,l_ef) #法向量
        
        cos_q6 = np.dot(dfe, l_cd) / (np.linalg.norm(dfe) * np.linalg.norm(l_cd))
        q6 = pi - math.acos(cos_q6)

        #q7
        # cos_beta = np.dot(l_cd, l_de) / (np.linalg.norm(l_cd) * np.linalg.norm(l_de))
        # cos_q7 = cos_beta/cos(q6-pi/2)
        # q7 = math.acos(cos_q7)
        #q7
        cos_q7 = np.dot(-l_d1d2, l_de) / (np.linalg.norm(l_d1d2) * np.linalg.norm(l_de))
        q7 = math.acos(cos_q7)-pi/2
        

        q = np.array([q1,q2,q3,q4,q5,q6,q7])
        return q



def Transform(DH_param):
    """
    这个函数主要用于计算相邻两个坐标系之间的变换矩阵
    param DH_param: DH参数,按照a,alpha,d,theta的顺序
    type DH_param: ndarray
    """

    a = DH_param[0]
    alpha = DH_param[1]
    d = DH_param[2]
    theta = DH_param[3]

    row1 = np.array([cos(theta),            -sin(theta),           0,            a             ])
    row2 = np.array([sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -d*sin(alpha)  ])
    row3 = np.array([sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha),   d*cos(alpha)  ])
    row4 = np.array([0,                     0,                     0,            1             ])

    T = np.vstack((row1, row2, row3, row4))

    return T



if __name__ == '__main__':
    robot = UPPERLIMB(l_b=0.3, l_s=0.4)
    l_h = 0.05
    T_hand = np.array([[1, 0, 0, l_h],       #把手掌当成了一个末端执行器
                     [0, 1, 0, 0  ],
                     [0, 0, 1, 0  ],
                     [0, 0, 0, 1  ]])
    q0 = np.array([0,    pi/2,  -pi/2,  0,    pi/2,  pi/2,  0]) #初始状态

    qx = np.array([0,0,0,0,0,0,0])
    qy = np.array([pi/2,0,0,0,0,0,0])
    # print(robot)      
    qt = rtb.jtraj(qx,qy,50)
    robot.plot(qt.q)
    T = robot.fkine(q0)
    
    print(T)
    # qx = np.array([0,0,0,0,0,0,0])
    # qy = np.array([pi/2,0,0,0,0,0,0])
    # Jacob = robot.jacob0(qx)
    # print(Jacob)