import math
import numpy as np


# angle1 = np.array([1,2,3,4,5,6,7,8,9]).reshape(3,3)

# a = angle1[1,]
# b = np.array([0,1,2]).reshape(1,3)
# c = np.array([2,1,0]).reshape(1,3)


# A_w = b
# u_x = -A_w[0,0]
# u_y = -A_w[0,1]
# u_z = 0
# u = np.array([[u_x, u_y, u_z]])
# # print(b.shape)
# print((c-b)/np.linalg.norm(c-b))
# print(u)

# a = np.array([2,0,0]).reshape(1,3)
# b = np.array([1,np.sqrt(3),0]).reshape(1,3)
# c = np.cross(a,b)
# print(c)
# print(c.shape)
# coor = np.array([a,b,c]).squeeze()
# print(coor.shape)

def tranCoordinate(p_w):
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

    #A_b = coor_base*A_w
    A_b = np.dot(coor_base, A_w)
    B_b = np.dot(coor_base, B_w)
    C_b = np.dot(coor_base, C_w)
    D_b = np.dot(coor_base, D_w)
    E_b = np.dot(coor_base, E_w)
    F_b = np.dot(coor_base, F_w)

    p_b = np.hstack([A_b,B_b,C_b,D_b,E_b,F_b])

    return p_b



p_w = np.array([
    [1, 5, -1, 0, 4, 1],
    [1, 2, 0,  1, 4, 7],
    [0, 0, 0,  3, 0, 3]
])


p_b = tranCoordinate(p_w)
print(p_b)

A = p_b[:,0].reshape(3,1)
B = p_b[:,1].reshape(3,1)
C = p_b[:,2].reshape(3,1)
D = p_b[:,3].reshape(3,1)
E = p_b[:,4].reshape(3,1)
F = p_b[:,5].reshape(3,1)
q2 = -math.acos(-C[2]/np.linalg.norm(C))
print(C.shape)