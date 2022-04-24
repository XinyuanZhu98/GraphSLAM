import numpy as np
import math
import matplotlib.pyplot as plt
from slam_functions import *
import random

f = open('SLAM_graph_nodes_odom1.dat', 'w')

meas_data = np.loadtxt("data_preprocessed.dat")
time_old = 0.0
x_old = 3.0197561
y_old = 7.0899048e-02
theta_old = -2.9101574
x = []
y = []
landmarks = []
lm_x_list = []
lm_y_list = []

counter = 0
for ele in meas_data:
    time_new = ele[1]
    landmarks.append(ele[2])
    if time_new != time_old:
        delta_t = time_new - time_old
        x_new = x_old + delta_t * math.cos(theta_old) * ele[5]
        y_new = y_old + delta_t * math.sin(theta_old) * ele[5]
        theta_new = wraptopi(theta_old + delta_t * ele[6])
        f.write(str(x_new) + '\n')
        f.write(str(y_new) + '\n')
        f.write(str(theta_new) + '\n')
        x.append(x_new)
        y.append(y_new)
        time_old = time_new
        x_old = x_new
        y_old = y_new
        theta_old = theta_new
    counter += 1

print(counter)
print(set(landmarks))
plt.scatter(x, y, c='peru', marker='o', s=2, label='robot positions')
plt.plot(x, y, c='peru')


for i in range(17):
    lm_x = 10 * random.random()
    lm_y = 4 * random.random() - 2
    lm_x_list.append(lm_x)
    lm_y_list.append(lm_y)
    f.write(str(lm_x) + '\n')       # x: [0, 10]
    f.write(str(lm_y) + '\n')       # y: [-2, 2]
plt.scatter(lm_x_list, lm_y_list, c='darkcyan', marker='x', s=4, label='landmark positions')
plt.legend(loc="upper right")
plt.savefig('init_odom.png')
f.close()

