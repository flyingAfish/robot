#!/usr/bin/env python
# coding=utf-8
"""
***********************************************
***********************************************
***********************************************
*****-------------------------------------*****
*****				h=59	              *****
*****-------------------------------------*****
*****				/	/				  *****
*****			   /   /                  *****
*****			  /	  /	                  *****
*****			 /	 /	hu=111.34         *****
*****			/   /                     *****
*****		   /   /                      *****
*****		  /	  /		                  *****
*****		 /	 /		                  *****
*****		 \   \                        *****
*****		  \   \                       *****
*****		   \   \                      *****
*****			\	\                     *****
*****			 \	 \  hl=136            *****
***** 			  \	  \					  *****
*****			   \   \                  *****
*****			   (    )                 *****
*****			   ‘----’                 *****
***********************************************
***********************************************
***********************************************
"""
import numpy as np
import math
import time
from matplotlib import animation
from matplotlib.widgets import Slider
from scipy import integrate
import matplotlib.pyplot as plt
# 解决中文乱码问题
plt.rcParams['font.sans-serif'] = ['SimHei']    # 用来设置字体样式以正常显示中文标签
plt.rcParams['axes.unicode_minus'] = False    # 默认是使用Unicode负号，设置正常显示字符，如正常显示负号


big_leg = 111.34
small_leg = 136
hip = 0

S = 100  # 距离
H = 50  # 高度
T = 2  # 周期
Tm = 1  # 摆动相周期
Tf = T - Tm  # 支撑相周期
pi = 3.14
phase = 0  # 相位
# t = (time.time() + phase) % T  # 时间
precision = 100
t = np.arange(precision * phase, precision * T + 1, 1) / precision  # 时间
n = 4
robot_hight = 220  # 機器人站立高度
foot_x_offset = S / 2  # 足端x轴方向相对髋部原点偏移量

print(t)

def xyztoang(x, y, z, yoffh, hu, hl):
    dyz = math.sqrt(y ** 2 + z ** 2)
    lyz = math.sqrt(dyz ** 2 - yoffh ** 2)
    gamma_yz = -math.atan(y / z)
    gamma_h_offset = -math.atan(-yoffh / lyz)
    gamma = gamma_yz - gamma_h_offset

    lxzp = math.sqrt(lyz ** 2 + x ** 2)
    n = (lxzp ** 2 - hl ** 2 - hu ** 2) / (2 * hu)
    beta = -math.acos(n / hl)

    alfa_xzp = -math.atan(x / lyz)
    alfa_off = math.acos((hu + n) / lxzp)
    alfa = alfa_xzp + alfa_off
    if any(np.isnan([gamma, alfa, beta])):
        print(x, y, z, yoffh, hu, hl)
    return [gamma, alfa, beta]


def hip_axes(gamma, beta, alfa, yoffh, hu, hl):
    n = hl * math.cos(beta)
    m = hl * math.sin(beta)
    lxzp = math.sqrt((hu + n) ** 2 + m ** 2)

    alfa_off = math.acos((hu + n) / lxzp)
    alfa_xzp = alfa - alfa_off
    x = lxzp * math.sin(alfa_xzp)
    y = -yoffh
    z = lxzp * math.cos(alfa_xzp)
    if any(np.isnan([x, y, z])):
        print(gamma, beta, alfa, yoffh, hu, hl)
    return [x, y, z]

def nick_axes(yoffh, alfa, hu):
    x = -hu*math.sin(alfa)
    y = -yoffh
    z = -hu*math.cos(alfa)
    return [x, y, z]

def metr_trajectory(t):
    # 摆动相曲线
    x = 0
    y = 0
    x_v = 0
    y_v = 0
    x_a = 0
    y_a = 0
    if 0 <= t < Tm:
        x = S * (t / Tm - 1 / 2 / pi * math.sin(2 * pi * t / Tm))
        x_v = S * (1/Tm - 1/Tm * math.cos(2 * pi * t / Tm))
        x_a = S*(2*pi/Tm/Tm*math.sin(2 * pi * t / Tm))

    if 0 <= t < Tm / 2:
        y = 2 * H * (t / Tm - 1 / n / pi * math.sin(n * pi * t / Tm))
        y_v = 2*H*(1/Tm - 1/Tm*math.cos(n * pi * t / Tm))
        y_a = 2*H*(n*pi/Tm/Tm*math.sin(n * pi * t / Tm))
    elif Tm / 2 <= t < Tm:
        y = 2 * H * (1 - t / Tm + 1 / n / pi * math.sin(n * pi * t / Tm))
        y_v = 2*H*(-1/Tm + 1/Tm*math.cos(n * pi * t / Tm))
        y_a = 2*H*(-n*pi/Tm/Tm*math.sin(n * pi * t / Tm))

    # 支撑相曲线
    if Tm <= t <= T:
        x = S * ((2 * Tf - t) / Tf + 1 / 2 / pi * math.sin(2 * pi * t / Tf))
        x_v = S*(-1/Tf + 1/Tf*math.cos(2 * pi * t / Tf))
        x_a = S*(-2*pi/Tf/Tf*math.sin(2 * pi * t / Tf))
        y = 0

    return x, y, x_v, y_v, x_a, y_a

def add_slider(pos, name, min, max):
    ax = plt.axes([0, 0.1 + pos, 0.65, 0.03], fc='lightgoldenrodyellow')
    slider = Slider(ax, name, min, max, valfmt='% .2f', valinit=50)
    def update(val):
        H = val
        print(val)
    slider.on_changed(update)

x_list = []
y_list = []
x_v_list = []
y_v_list = []
x_a_list = []
y_a_list = []
alfa_list = []
beta_list = []
def generate_plot_param():
    for i in t:
        x_metr, y_metr, x_v_metr, y_v_metr, x_a_metr, y_a_metr = metr_trajectory(i)
        x_list.append(x_metr)
        y_list.append(y_metr-robot_hight)
        x_v_list.append(x_v_metr)
        y_v_list.append(y_v_metr)
        x_a_list.append(x_a_metr)
        y_a_list.append(y_a_metr)

        gamma, alfa, beta = xyztoang(x_metr, 0, y_metr - robot_hight,
                                     yoffh=hip, hu=big_leg, hl=small_leg)
        alfa_list.append(alfa)
        beta_list.append(beta)
    print("alfa_list:", alfa_list)
    print("beta_list:", beta_list)

nick_x_list = []
nick_z_list = []
def anima_trajectory_param():
    density = 0.05
    len_list = len(x_list)
    indx = 0
    i = 0
    inc = math.ceil(density * len_list / 2)
    while indx <= len_list:
        nick_x_list.append(list())
        nick_z_list.append(list())
        if len_list/2 - indx - inc < inc/2 and len_list/2 - indx > 0:
            rel_indx = math.ceil(len_list / 2)
            indx = indx + inc
        elif len_list - indx - inc < inc/2:
            rel_indx = len_list - 1
            indx = indx + inc
        else:
            rel_indx = indx
        print(len_list / 2 - indx, inc, rel_indx, indx)
        gamma, alfa, beta = xyztoang(x_list[rel_indx], 0, y_list[rel_indx],
                                     yoffh=hip, hu=big_leg, hl=small_leg)
        nick_x, nick_y, nick_z = nick_axes(yoffh=hip, alfa=alfa, hu=big_leg)
        nick_x_list[i].append(foot_x_offset)
        nick_x_list[i].append(nick_x)
        nick_x_list[i].append(x_list[rel_indx])
        nick_z_list[i].append(foot_x_offset)
        nick_z_list[i].append(nick_z)
        nick_z_list[i].append(y_list[rel_indx])
        indx = indx + inc
        i = i + 1

def draw_sva_plot():
    plt.figure(1)
    plt.subplot(231)
    plt.title(label='s-t')
    plt.plot(t, x_list, label='x')
    plt.plot(t, y_list, label='y')
    plt.legend(loc=0, ncol=1)
    plt.grid()
    plt.xlabel(xlabel='t(s)')
    plt.ylabel(ylabel='s(cm)')
    plt.subplots_adjust(bottom=0.8)

    plt.subplot(232)
    plt.title(label='v-t')
    plt.plot(t, x_v_list, label='x')
    plt.plot(t, y_v_list, label='y')
    plt.legend(loc=0, ncol=1)
    plt.grid()
    plt.xlabel(xlabel='t(s)')
    plt.ylabel(ylabel='v(cm/s)')

    plt.subplot(233)
    plt.title(label='a-t')
    plt.plot(t, x_a_list, label='x')
    plt.plot(t, y_a_list, label='a')
    plt.legend(loc=0, ncol=1)
    plt.grid()
    plt.xlabel(xlabel='t(s)')
    plt.ylabel(ylabel='a(cm/s/s)')

    plt.subplot(212)
    plt.title(label='y-x')
    plt.plot(x_list, y_list)
    plt.grid()
    plt.xlabel(xlabel='x(cm)')
    plt.ylabel(ylabel='s(cm)')
    plt.tight_layout()

def draw_anim_plt():
    fig = plt.figure(2)
    plt.title(label='y-x')
    plt.plot(x_list, y_list)
    # for i in range(len(nick_x_list)):
    #     plt.plot(nick_x_list[i], nick_z_list[i])
    l, = plt.plot(nick_x_list[0], nick_z_list[0])
    plt.grid()
    plt.xlabel(xlabel='x(cm)')
    plt.ylabel(ylabel='y(cm)')
    # plt.tight_layout()
    print(nick_x_list)
    def update(i):
        l.set_xdata(nick_x_list[i-1])
        l.set_ydata(nick_z_list[i-1])
        return l,
    ani = animation.FuncAnimation(fig=fig,
                                  func=update,
                                  frames=len(nick_x_list),
                                 # init_func=init,
                                  interval=500,
                                  blit=True)

def draw_joint_trend_plot():
    plt.figure(3)
    plt.title(label=u'四足机器人关节角度变化趋势图')
    plt.subplot(211)
    plt.plot(t, alfa_list, label='大腿角度')
    plt.plot(t, beta_list, label='小腿角度')
    plt.subplot(212)
    plt.plot(t, x_list, label='足端位移')
    plt.plot(t, y_list, label='足端高度')
    plt.grid(linestyle='--')
    plt.xlabel(xlabel='rad(o)')
    plt.ylabel(ylabel='t(s)')
    plt.legend()
    plt.xlim(0, 2)
    # plt.ylim(-180, 180)
    # plt.yticks(np.arange(0, 410, 10) - 240)
    plt.tight_layout()

def show_plot():
    plt.show()

if __name__ == '__main__':
    generate_plot_param()
    anima_trajectory_param()
    draw_sva_plot()
    draw_anim_plt()
    draw_joint_trend_plot()
    show_plot()

