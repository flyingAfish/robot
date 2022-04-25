import numpy as np
import matplotlib.pyplot as plt
from scipy import integrate
from matplotlib import animation
import time
import math
# 解决中文乱码问题
plt.rcParams['font.sans-serif'] = ['SimHei']    # 用来设置字体样式以正常显示中文标签
plt.rcParams['axes.unicode_minus'] = False    # 默认是使用Unicode负号，设置正常显示字符，如正常显示负号

big_leg = 111.34
small_leg = 136
hip = 0
def foot_axes(gamma, beta, alfa, yoffh, hu, hl):
    n = hl * math.cos(beta)
    m = hl * math.sin(beta)
    lxzp = math.sqrt((hu + n) ** 2 + m ** 2)

    alfa_off = math.acos((hu + n) / lxzp)
    alfa_xzp = alfa - alfa_off
    x = lxzp * math.sin(alfa_xzp)
    y = -yoffh
    z = -lxzp * math.cos(alfa_xzp)
    if any(np.isnan([x, y, z])):
        print(gamma, beta, alfa, yoffh, hu, hl)
    return [x, y, z]

def knee_axes(yoffh, alfa, hu):
    x = -hu*math.sin(alfa)
    y = -yoffh
    z = -hu*math.cos(alfa)
    return [x, y, z]

# 0.027
class Cpg(object):
    def __init__(self, alpha=100, a=50,
                 mu=1, beta=0.5, omega_sw=2 * np.pi,
                 time_steps=0.01, p0=[1, 0]):
        """
        初始化参数
        """
        self.time_steps = time_steps
        self.alpha = alpha
        self.a = a
        self.mu = mu
        self.beta = beta
        self.omega_sw = omega_sw
        self.p0 = p0

    def hopf(self, pos, dt):
        x, y = pos
        alpha, a, mu, beta, omega_sw = self.get_parms()
        r_square = x ** 2 + y ** 2
        omega_st = ((1 - self.beta) / self.beta) * self.omega_sw
        omega = omega_st / (math.e ** (-a * y) + 1) + omega_sw / (math.e ** (a * y) + 1)
        dx = (alpha * (mu - r_square) * x - omega * y) * dt
        dy = (alpha * (mu - r_square) * y + omega * x) * dt
        return [x + dx, y + dy]

    def get_parms(self):
        return self.alpha, self.a, self.mu, self.beta, self.omega_sw

    def calculate(self, t):
        """
        调用振荡器函数，返回数据序列
        """
        data = []
        data_x = []
        data_y = []
        pos = self.p0
        for i in t:
            pos = self.hopf(pos=pos, dt=self.time_steps)
            data_x.append(pos[0])
            data_y.append(pos[1])
        data.append(data_x)
        data.append(data_y)
        return data

    def hip_map(self, beta):
        return beta

    def knee_map(self, alfa):
        return alfa

    def generate_foot_datas(self, betas: list, alfas: list):
        x_list = []
        z_list = []
        for i in range(0, len(betas)):
            x, y, z = foot_axes(0, betas[i], alfas[i], hip, big_leg, small_leg)
            x_list.append(x)
            z_list.append(z)
        return x_list, z_list

    def generate_leg_datas(self, betas: list, alfas: list):
        x_list = []
        z_list = []
        for i in range(0, len(betas)):
            x_list.append(list())
            z_list.append(list())
            foot_x, foot_y, foot_z = foot_axes(0, betas[i], alfas[i], hip, big_leg, small_leg)
            knee_x, knee_y, knee_z = knee_axes(hip, alfas[i], big_leg)
            x_list[i].append(foot_x)
            x_list[i].append(knee_x)
            x_list[i].append(0)
            z_list[i].append(foot_z)
            z_list[i].append(knee_z)
            z_list[i].append(0)
        return x_list, z_list

    def show(self):
        """
        将信号画出来
        """
        t = np.arange(0, 20, self.time_steps)
        t0 = time.time()
        data = self.calculate(t)
        print(data)
        t1 = time.time()
        print('time: ', t1 - t0)

        fig1 = plt.figure()
        plt.title(label='a-t')
        plt.plot(t, data[0], label='x')
        plt.plot(t, data[1], label='y')
        plt.legend(loc=0, ncol=1)
        plt.grid()
        plt.xlabel(xlabel='时间')
        plt.ylabel(ylabel='幅值')

        #
        # fig2 = plt.figure()
        # plt.axis('equal')
        # plt.plot(data[0], data[1])

    def drew_robot(self):
        """
        将信号画出来
        """
        t = np.arange(0, 20, self.time_steps)
        t0 = time.time()

        # 1.cpg网络生成角度值
        data = self.calculate(t)
        betas = self.hip_map(data[0])
        alfas = self.knee_map(data[1])

        # 2.生成足端轨迹数据
        foot_x_list, foot_z_list = self.generate_foot_datas(betas, alfas)

        # 3.生成腿部轨迹数据：大腿和小腿
        leg_x_list, leg_z_list = self.generate_leg_datas(betas, alfas)

        t1 = time.time()
        print('time: ', t1 - t0)

        fig = plt.figure(2)
        plt.title(label='四足机器人单腿仿真')
        # 4.绘制足端轨迹
        plt.plot(foot_x_list, foot_z_list)

        plt.grid()
        plt.xlabel(xlabel='时间')
        plt.ylabel(ylabel='幅值')

        # 初始化腿部点
        l, = plt.plot(leg_x_list[0], leg_z_list[0])

        # 更新腿部点位
        def update(i):
            l.set_xdata(leg_x_list[i - 1])
            l.set_ydata(leg_z_list[i - 1])
            return l,

        # 5.动态绘制腿部轨迹
        ani = animation.FuncAnimation(fig=fig,
                                      func=update,
                                      frames=len(leg_x_list),
                                      # init_func=init,
                                      interval=50,
                                      blit=True)


if __name__ == '__main__':
    signal = Cpg(alpha=10, a=30,
                 mu=1, beta=0.25, omega_sw=1,
                 time_steps=0.01, p0=[0.01, 1])
    signal.show()
    #
    # signal = Cpg(alpha=10, a=30,
    #              mu=4, beta=0.5, omega_sw=1,
    #              time_steps=0.01, p0=[0.01, 0])
    # signal.show()
    # signal = Cpg(alpha=10, a=30,
    #              mu=9, beta=0.75, omega_sw=1,
    #              time_steps=0.01, p0=[0.01, 0])
    # signal.show()

    signal.drew_robot()
    plt.show()