import numpy as np
import matplotlib.pyplot as plt
from scipy import integrate
import time
import math

# 参考地址：https://blog.csdn.net/qq_48971199/article/details/123553137
def get_param():
    alpha = 10 #收敛速度
    leg_num = 4 #腿的数量
    gait = 2 #步态选择
    mu = 1
    a = 50
    psai = [1, 1, -1, -1] #关节形式 膝式-1 肘式 1
    omega_sw = 2*np.pi
    u1=0
    u2=0 #误差 影响x,y平衡位置
    h = 0.02 #抬腿高度
    v = 1 #行走速度
    gait_t = 0.4#步态周期
    s = v*gait_t
    l = 0.4 #腿节长度
    theta0 = math.radians(30)#髋关节和膝关节平衡位置与垂直线夹角
    L = 2*l*np.cos(theta0)#髋关节与足端之间长度
    ah = 15 #np.arcsin((beta*s)/(2*L)) 髋关节摆动幅度
    ak =10 #膝关节摆动幅度
    return [alpha,leg_num,gait,mu,a,psai,omega_sw,u1,u2]
def get_gait(gait_m):
    #walk
    global beta, phase, v, t, h, ah, ak
    if gait_m == 1:
        beta=0.75
        phase=[0,0.5,0.25,0.75]
        v=0.3
        t=0.8
        h=0.02
        ah=7.5
        ak=5.3
    #trot
    elif gait_m == 2:
        beta=0.5
        phase=[0,0.5,0,0.5]
        v = 1
        t = 0.4
        h = 0.02
        ah = 8.3
        ak = 5.3
    #pace
    elif gait_m == 3:
        beta=0.5
        phase=[0,0.5,0.5,0]
        v = 1.2
        t = 0.4
        h = 0.02
        ah = 10
        ak = 5.3
    #gallop
    elif gait_m == 4:
        beta = 0.5
        phase = [0, 0, 0.5, 0.5]
        v = 1.6
        t = 0.4
        h = 0.04
        ah = 13.4
        ak = 10
    return[ beta , phase , v , t , h , ah , ak ]

def get_theta():

    theta = np.zeros([4, 4])
    for i in range(4):
        for j in range(4):
            theta[i, j] = (phase[i] - phase[j])
    return 2 * np.pi * theta


def get_r(x, y):
    r_x = np.zeros([4, 4])
    r_y = np.zeros([4, 4])
    theta = get_theta()
    for i in range(4):
        for j in range(4):
            r_x[i, j] = np.cos(theta[i, j]) * x[j] + np.sin(theta[i, j]) * y[j]
            r_y[i, j] = -np.sin(theta[i, j]) * x[j] + np.cos(theta[i, j]) * y[j]
    return np.sum(r_x, axis=1), np.sum(r_y, axis=1)
def hopf(gait_m,pos,steps,time):
    beta, phase, v, t, h, ah, ak = get_gait(gait_m)
    alpha, leg_num, gait, mu, a, psai, omega_sw, u1, u2 = get_param()
    x1,x2,x3,x4,y1,y2,y3,y4 = pos
    x= np.array([x1,x2,x3,x4])
    y= np.array([y1,y2,y3,y4])
    omega_st = ((1 - beta) / beta) * omega_sw
    # T = np.pi / omega_st + np.pi / omega_sw
    x = np.array([x1, x2, x3, x4])
    y = np.array([y1, y2, y3, y4])
    num=len(time)
    # r = x ** 2 + y ** 2
    # omega = omega_st / (np.e ** (-100 * y) + 1) + omega_sw / (np.e ** (100 * y) + 1)
    # r_x, r_y = get_r(x, y)
    # dx = alpha * (mu - r) * x - omega * y + r_x
    # dy = alpha * (mu - r) * y + omega * x + r_y
    dx=np.zeros([4])
    dy=np.zeros([4])
    leg_h_Point_x = np.zeros([num, leg_num])
    leg_h_Point_y = np.zeros([num, leg_num])
    leg_k_Point_x = np.zeros([num, leg_num])
    leg_k_Point_y = np.zeros([num, leg_num])
    Foot_end_x    = np.zeros([num, leg_num])
    Foot_end_y    = np.zeros([num, leg_num])
    for i in range(num):
        r = (x - u1) ** 2 + (y - u2) ** 2
        omega = omega_st / (np.e ** (-100 * y) + 1) + omega_sw / (np.e ** (100 * y) + 1)
        r_x, r_y = get_r(x, y)
        dx = alpha * (mu - r) * x - omega * y + r_x
        dy = alpha * (mu - r) * y + omega * x + r_y
        x = x + dx * steps
        y = y + dy * steps
        # print(x,y)
        leg_h_Point_x[i] = x
        leg_h_Point_y[i] = y
        # print(leg_h_Point_x[i,j],leg_h_Point_y[i,j])
        for j in range(4):
            if y[j] > 0:
                leg_k_Point_y[i, j] = 0
            else:
                leg_k_Point_y[i, j] = -np.sign(psai[j]) * (ak / ah) * y[j]

    pos = np.hstack([leg_h_Point_x, leg_h_Point_y])
    date = np.hstack([pos, leg_k_Point_y])
    return date
p0 = [1, -1, -1, 1, 0, 0, 0, 0]
time = np.arange(0,20,0.001)
date = hopf(1,p0,0.001,time)
print(date.shape)
plt.subplot(4, 1, 1)
plt.title('figure1')
plt.plot(time, date[:, 0], "-r", label="x")
plt.plot(time, date[:, 8], "-b", label="y")
# print(date[:,0],date[:,4])
plt.legend(loc="right")
# plt.show()
plt.subplot(4, 1, 2)
# plt.title('figure2')
plt.plot(time, date[:, 1], "-r", label="x")
plt.plot(time, date[:, 9], "-b", label="y")
# print(date[:, 0], date[:, 4])
plt.legend(loc="right")
# plt.show()
plt.subplot(4, 1, 3)
# plt.title('figure3')
plt.plot(time, date[:, 2], "-r", label="x")
plt.plot(time, date[:, 10], "-b", label="y")
# print(date[:, 0], date[:, 4])
plt.legend(loc="right")
# plt.show()
plt.subplot(4, 1, 4)
# plt.title('figure4')
plt.plot(time, date[:, 3], "-r", label="x")
plt.plot(time, date[:, 11], "-b", label="y")
# print(date[:, 0], date[:, 4])
plt.legend(loc="right")
plt.show()

plt.title('fig')
plt.plot(time, date[:, 0], "-r", label="x1")
plt.plot(time, date[:, 1], "-b", label="x2")
plt.plot(time, date[:, 2], "-y", label="x3")
plt.plot(time, date[:, 3], "-g", label="x4")

plt.legend(loc="right")
plt.show()
