#!/usr/bin/env python
# -*- coding:utf-8 -*-
import csv 
import os
import numpy as np
import math

def MiniNM(A,B):
    """
    求解Ax = B

    """
    if not (isinstance(A, np.ndarray) and isinstance(B, np.ndarray)):
        print("the type of params is not np.ndarray")
        print("the type of A is ",type(A))
        print("the type of B is ",type(B))
    elif A.shape[0] != B.shape[0]:
        print("False: A and B have different numbers of rows")
    else:
        q, r = np.linalg.qr()
        p = np.dot(q.T, B)
        x = np.dot(np.linalg.pinv(r), p)
        print("x =", x)
        return x


def ibk(f, x):
    """
    使用最小二乘法，通过力信号和位移信号求解手臂的末端刚度（以及质量、阻尼）

    param f: 力信号
    type f: ndarray(nx3)
            [[fx1, fy1, fz1],
             [fx2, fy2, fz2],
             ......         ]
    param l: 各个维度下的位移、速度、加速度信号
    type l: ndarray(nx9)

    return:
    i:所求系统的虚拟质量
    b:所求系统的阻尼
    k:所求系统的刚度
    type i,b,k: ndarray(3x3)
    """
    z = MiniNM(A=x,B=f)
    i = z[0:3,:]
    b = z[3:6,:]
    k = z[6:9,:]

    return i,b,k






if __name__ == "__main__":

    pass