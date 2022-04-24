
import matplotlib.pyplot as plt
from slam_classes import *

plt.figure(1)
i = 0
x = []
y = []
time_size = 2000
with open('x_gt.dat') as rb_x_gt:
    for line in rb_x_gt.readlines():
        i += 1
        if i < time_size:
            x.append(float(line))
rb_x_gt.close()

j = 0
with open('y_gt.dat') as rb_y_gt:
    for line in rb_y_gt.readlines():
        j += 1
        if j < time_size:
            y.append(float(line))
rb_y_gt.close()

plt.scatter(x, y, c='r', marker='o', s=4)
print('Robot poses plotting finished!')

with open('landmark_gt.dat') as lm_gt:
    for line in lm_gt.readlines():
        temp = line.split()
        plt.scatter(float(temp[0]), float(temp[1]), c='b', marker='o', s=4)
lm_gt.close()
print('Landmark positions plotting finished!')

plt.title('Groundtruth')
plt.xlabel('x[m]')
plt.ylabel('y[m]')
plt.savefig('gt_dense.png')

