#!/usr/bin/env python
#coding=utf-8

import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

if __name__ == '__main__' :
  # 定义结构参数
  b = 4
  w = 8
  l = 8
  # 定义目标位姿
  pos = np.mat([0.0,  4.0,  3.0 ]).T # 目标位置向量
  rpy = np.array([0.0,  0.0,  0.0]) * math.pi / 180 # 欧拉角，化为弧度值
  # 将欧拉角转换为旋转矩阵
  R, P, Y = rpy[0], rpy[1], rpy[2]
  rotx = np.mat([[ 1,       0,            0          ],
                 [ 0,       math.cos(R), -math.sin(R)],
                 [ 0,       math.sin(R),  math.cos(R)]])
  roty = np.mat([[ math.cos(P),  0,      -math.sin(P)],
                 [ 0,            1,       0          ],
                 [ math.sin(P),  0,       math.cos(P)]])
  rotz = np.mat([[ math.cos(Y), -math.sin(Y),  0     ],
                 [ math.sin(Y),  math.cos(Y),  0     ],
                 [ 0,            0,            1     ]])
  rot_mat = rotx * roty * rotz
  # 结构参数
  body_struc = np.mat([[ l / 2,  b / 2,  0],
                       [ l / 2, -b / 2,  0],
                       [-l / 2,  b / 2,  0],
                       [-l / 2, -b / 2,  0]]).T
  footpoint_struc = np.mat([[ l / 2,  w / 2,  0],
                            [ l / 2, -w / 2,  0],
                            [-l / 2,  w / 2,  0],
                            [-l / 2, -w / 2,  0]]).T
  # 计算单腿末端位置向量AB
  AB = np.mat(np.zeros((3, 4)))
  for i in range(4):
    AB[:, i] = - pos - rot_mat * body_struc[:, i] + footpoint_struc[:, i]
  print(AB)

  OA = body_struc + pos
  OB = footpoint_struc

  OA_FOR_LINE = np.concatenate((OA[:,0:2], OA[:,3], OA[:,2], OA[:,0]), axis=1)
  AB1_LINE = np.concatenate((OA[:, 0], OB[:, 0]), axis=1)
  AB2_LINE = np.concatenate((OA[:, 1], OB[:, 1]), axis=1)
  AB3_LINE = np.concatenate((OA[:, 2], OB[:, 2]), axis=1)
  AB4_LINE = np.concatenate((OA[:, 3], OB[:, 3]), axis=1)

  x1 = OA_FOR_LINE[0, :]
  y1 = OA_FOR_LINE[1, :]
  z1 = OA_FOR_LINE[2, :]

  # 画图 定义图像和三维格式坐标轴
  fig = plt.figure()
  ax = Axes3D(fig)
  # ax.plot_wireframe(x , y ,z)
  ax.plot_wireframe(x1, y1, z1)
  ax.plot_wireframe(AB1_LINE[0, :], AB1_LINE[1, :], AB1_LINE[2, :])
  ax.plot_wireframe(AB2_LINE[0, :], AB2_LINE[1, :], AB2_LINE[2, :])
  ax.plot_wireframe(AB3_LINE[0, :], AB3_LINE[1, :], AB3_LINE[2, :])
  ax.plot_wireframe(AB4_LINE[0, :], AB4_LINE[1, :], AB4_LINE[2, :])

  ax.set_xlabel('x axis')
  ax.set_ylabel('y axis')
  ax.set_zlabel('z axis')

  plt.show()