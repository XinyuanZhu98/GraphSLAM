import numpy as np
import math
import matplotlib.pyplot as plt
from slam_functions import *
import random


time_old = 0.0
x_old = 3.0197561
y_old = 7.0899048e-02
theta_old = -2.9101574
vel_for_old = 0
vel_ang_old = 0
x = []
y = []
theta = []
landmarks = []
counter = 0

path = r"C:/Users/Xinyuan Zhu/PycharmProjects/ECE1505_Proj_2000/SLAM_graph_nodes_rand.dat"

info = []
lm_x = []
lm_y = []
time_len = 1986
with open(path) as res:
    for line in res.readlines():
        info.append(float(line))
res.close()

for k in range(time_len * 3):
    if k % 3 == 0:
        x.append(float(info[k]))
        y.append(float(info[k + 1]))
        theta.append(wraptopi(float(info[k + 2])))
plt.plot(x, y, c='peru', label='robot positions')

for m in range(34):
    if m % 2 == 0:
        lm_x.append(float(info[time_len * 3 + m]))
        lm_y.append(float(info[time_len * 3 + m + 1]))
plt.scatter(lm_x, lm_y, c='darkcyan', marker='x', s=4, label='landmark positions')
plt.title('0')
plt.xlabel('x[m]')
plt.ylabel('y[m]')
plt.legend(loc='upper right')
plt.savefig('init_rand.png')
# plt.show()