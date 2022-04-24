import numpy as np
import math
import matplotlib.pyplot as plt
from slam_functions import *
import random

f = open('SLAM_graph_nodes_rand_input.dat', 'w')

meas_data = np.loadtxt("data_preprocessed.dat")
time_old = 0.0
x_old = 3.0197561
y_old = 7.0899048e-02
theta_old = -2.9101574
vel_for_old = 0
vel_ang_old = 0
x = []
y = []


for ele in meas_data:
    time_new = ele[1]
    if time_new != time_old:
        delta_t = time_new - time_old
        u = random.random()
        omega = random.random()
        x_new = x_old + delta_t * math.cos(theta_old) * u
        y_new = y_old + delta_t * math.sin(theta_old) * u
        theta_new = wraptopi(theta_old + delta_t * omega)
        f.write(str(x_new) + '\n')
        f.write(str(y_new) + '\n')
        f.write(str(theta_new) + '\n')
        x.append(x_new)
        y.append(y_new)
        time_old = time_new
        x_old = x_new
        y_old = y_new
        theta_old = theta_new

plt.scatter(x, y, c='peru', marker='o', s=2, label='robot positions')
plt.plot(x, y, c='peru')


lm_x = []
lm_y = []
for i in range(17):
    lm_x_rand = 10 * random.random()
    lm_y_rand = 4 * random.random() - 2
    lm_x.append(lm_x_rand)
    lm_y.append(lm_y_rand)
    f.write(str(lm_x_rand) + '\n')       # x: [0, 10]
    f.write(str(lm_y_rand) + '\n')    # y: [-2, 2]
f.close()

plt.scatter(lm_x, lm_y, c='darkcyan', marker='x', s=4, label='landmark positions')

plt.title('0')
plt.xlabel('x[m]')
plt.ylabel('y[m]')
plt.legend(loc='upper right')
plt.savefig('init_rand_input.png')

