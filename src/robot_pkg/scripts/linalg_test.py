import numpy as np



def ibk(l, f):
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

    q,r = np.linalg.qr(l)
    p = np.dot(q.T, f)
    z = np.dot(np.linalg.pinv(r), p)

    i = z[0:3,:]
    b = z[3:6,:]
    k = z[6:9,:]

    return i,b,k


if __name__ == "__main__":


    A = np.array([[1,0,1,0,1,0,0,0,1],
                [0,1,0,0,0,1,1,0,0],
                [0,0,1,1,0,1,0,1,0]])
    # A = np.eye(3,3)
    B = np.array([[1,0],
                [0,1],
                [0,0]])

    i,b,k = ibk(A, B)
    print(i,b,k)
    if not isinstance(i,np.ndarray):
        print(isinstance(i,np.ndarray))