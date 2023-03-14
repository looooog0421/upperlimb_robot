import numpy as np
import math
import random
from math import pi
from scipy.signal import spectrogram,windows,get_window, argrelextrema
from scipy.fftpack import fft,fftshift,ifft
from scipy.fftpack import fftfreq
import matplotlib.pyplot as plt

def disturbance(ta, fc=50,fl=0,fh=5):
    """
    这个函数用于生成理想幅值谱函数

    param fc: 协作机械臂的控制频率
    type fc: int(这里默认使用ur5e机械臂,其控制频率为500Hz,但是500Hz过大,暂时考虑使用50Hz)
    param ta: 计划产生扰动的总持续时间
    type ta: double or int
    """
    
    fh = 5 #频率上限
    fl = 0 #频率下限
    num = fc*ta
    #ur5e机械臂控制频率是500Hz
    A_n = np.zeros(int(ta*fc)).reshape(1,-1)
    A_n[0,0:ta*(fh-fl)] = 1
    return A_n,num


def randomPhase(N, num):
    """
    利用随机数生成服从区间(-pi.pi)的单边随机相位谱函数
    
    param N: 需要测量的末端刚度维度
    type N: int (1 or 2 or 3)
    param num: 频谱图上的点数量
    type num: int (一般为偶数)
    """
    ran_array = []
    for i in range(N):
        ran_list = []
        for j in range(int(num/2)):
            ran_num = random.uniform(-pi,pi)
            ran_list.append(ran_num)
        ran_array.append(ran_list)
    phi = np.array(ran_array).reshape((N, int(num/2)))

    return phi

def doubleRandom(N, num):
    """
    从单边随机相位谱函数构建双边随机相位谱

    param: 单边随机相位谱函数
    type: ndarray(N x num/2)
    """
    phi = randomPhase(N, num)

    phi_another = np.zeros(phi.shape)
    # print(phi.shape)
    for i in range(phi.shape[1]):
        phi_another[:,-i-1] = -phi[:,i]

    phi_n = np.hstack((phi, phi_another))

    return phi_n

def spectrum(A_n, phi_n):
    """
    基于相位谱和幅值谱生成复频谱

    param ta: 计划产生扰动的总持续时间
    type ta: double or int
    """
    # print("phi_n = ",phi_n[0,:5])
    x = np.exp((0+1j)*phi_n)
    # print("x = ",x[0,:5])
    D_n = A_n*x
    # print(D_n)
    
    return D_n

def ifft_time(D_n):
    """
    求解反快速傅立叶变换
    """
    d_n = []
    for i in range(D_n.shape[0]):
        d_i = ifft(D_n[i,:])
        d_n.append(d_i)
        
    d_n = np.array(d_n).reshape(3,-1)
    # print("d_n = ",d_i[0:5])
    return d_n

def normalize(d_n,d_max = 10):
    """
    求解归一化,返回归一化后的极值点及其所在的序列
    """
    dn_norm = np.linalg.norm(d_n, axis=0)
    dn_norm = np.array(dn_norm).reshape(1,-1)
    extra = select(dn_norm)
    # print(dn_norm.shape)
    # print(d_n.shape)
    norm_max = np.max(dn_norm)
    # print(dn_norm)
    d_scale = d_n * d_max / norm_max
    d_select = d_scale[:, extra].squeeze()
    # print(extra.shape)
    d_extra = np.vstack((extra, d_select))

    return d_extra


def select(dn_norm):
    """
    挑选关键路径,返回极值点所在序列号
    """
    dn_norm = dn_norm.flatten()
    greater = argrelextrema(dn_norm, np.greater)
    less = argrelextrema(dn_norm, np.less)
    extra = np.hstack((greater,less))
    extra = np.sort(extra)
    # print(extra)
        

    return extra

    



def stiff(ta,N):
    """
    根据所需的维数和时间长度生成随机的扰动序列

    param ta: 计划产生扰动的总持续时间
    type ta: double or int
    param N: 需要测量的扰动序列维数
    type N: int(1 or 2 or 3)
    """
    A_n, num = disturbance(ta)
    phi_n = doubleRandom(N, num)

    D_n = spectrum(A_n, phi_n)

    d_n = ifft_time(D_n)
    # print(d_n)
    # dn_real = abs(d_n)
    dn_real = d_n.real

    # print(dn_real.shape)
    dn_nor = normalize(dn_real)
    return dn_nor,dn_real

if __name__ == "__main__":


    d_n,dn_real = stiff(ta=20,N=3)
    # # # print(d_n.shape)
    Dn_real = fft(dn_real[0,:])
    Dn_real = np.abs(Dn_real)
    # ps = Dn_real**2 / 64

    # # # x = fftfreq(50, float(1)/50)
    # # plt.plot(2*Dn_real[:500])
    # # plt.show()
    # # print(Dn_real[0])
    plt.plot(d_n[0],d_n[1])
    plt.show()

    # plt.plot(d_n[0,:],d_n[1,:])
    # plt.show()

    # #这里假设采样频率为100，时间为10,所以设置1000个点
    # random_array = np.random.rand(1000) / np.sqrt(1000)
    # Xm = fft(random_array)
    # print(Xm)
    # Xm = abs(Xm)
    # plt.plot(Xm)
    # plt.show()


