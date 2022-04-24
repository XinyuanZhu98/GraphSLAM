import numpy as np
import math
import matplotlib.pyplot as plt
from slam_functions import *
import random

meas_data = np.loadtxt("data_preprocessed.dat")
time_old = 0.0
x_old = 3.0197561
y_old = 7.0899048e-02
theta_old = -2.9101574
vel_for_old = 0
vel_ang_old = 0
x = []
y = []
landmarks = []
counter = 0
for ele in meas_data:
    time_new = ele[1]
    landmarks.append(ele[2])
    if time_new != time_old:
        delta_t = time_new - time_old
        x_new = x_old + delta_t * math.cos(theta_old) * ele[5]
        y_new = y_old + delta_t * math.sin(theta_old) * ele[5]
        theta_new = wraptopi(theta_old + delta_t * vel_ang_old)
        x.append(x_new)
        y.append(y_new)
        time_old = time_new
        x_old = x_new
        y_old = y_new
        theta_old = theta_new
        vel_for_old = ele[5]
        vel_ang_old = ele[6]
    counter += 1

print(counter)
print(set(landmarks))
plt.scatter(x, y, c='peru', marker='o', s=2, label='robot positions')
plt.plot(x, y, c='peru')

path = r"C:/Users/Xinyuan Zhu/PycharmProjects/ECE1505_Proj_2000/SLAM_graph_nodes_odom.dat"

info = []
lm_x = []
lm_y = []
time_len = 1986
with open(path) as res:
    for line in res.readlines():
        info.append(float(line))
res.close()

for m in range(34):
    if m % 2 == 0:
        lm_x.append(float(info[time_len * 3 + m]))
        lm_y.append(float(info[time_len * 3 + m + 1]))
plt.scatter(lm_x, lm_y, c='darkcyan', marker='x', s=4, label='landmark positions')
plt.title('0')
plt.xlabel('x[m]')
plt.ylabel('y[m]')
plt.legend(loc='upper right')
plt.savefig('init_odom.png')
