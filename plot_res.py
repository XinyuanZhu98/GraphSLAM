
import matplotlib.pyplot as plt
from slam_classes import *

odom_path = r"C:/Users/Xinyuan Zhu/PycharmProjects/ECE1505_Proj_2000/SLAM_graph_nodes_odom.dat"
res_path = r"C:/Users/Xinyuan Zhu/PycharmProjects/ECE1505_Proj_2000/result_rand/res_rand.txt"
save_path = r"C:/Users/Xinyuan Zhu/PycharmProjects/ECE1505_Proj_2000/traj_odom/"

plt.figure(1)
pos = []
time_len = 1987
with open(res_path) as res:
    for line in res.readlines():
        pos.append(float(line))
res.close()

pos_odom = []
with open(odom_path) as odom:
    for line in odom.readlines():
        pos_odom.append(float(line))
odom.close()

x = []
y = []
theta = []
odom_x = []
odom_y = []
odom_theta = []
lm_x = []
lm_y = []
for k in range(time_len * 3):
    if k % 3 == 0:
        x.append(float(pos[k]))
        y.append(float(pos[k + 1]))
        theta.append(wraptopi(float(pos[k + 2])))

plt.plot(x, y, c='peru', label='Graph SLAM result')
print('Results plotting finished!')

for p in range(time_len * 3):
    if p % 3 == 0:
        odom_x.append(float(pos_odom[p]))
        odom_y.append(float(pos_odom[p + 1]))
        odom_theta.append(wraptopi(float(pos_odom[p + 2])))

plt.plot(odom_x, odom_y, c='silver', label='Odometry')


for m in range(34):
    if m % 2 == 0:
        lm_x.append(float(pos[time_len * 3 + m]))
        lm_y.append(float(pos[time_len * 3 + m + 1]))
plt.scatter(lm_x, lm_y, c='peru', marker='o', s=4)

i = 0
x_gt = []
y_gt = []
theta_gt = []
lm_x_gt = []
lm_y_gt = []
with open('x_gt.dat') as rb_x_gt:
    for line in rb_x_gt.readlines():
        i += 1
        if i <= time_len:
            x_gt.append(float(line))
rb_x_gt.close()

j = 0
with open('y_gt.dat') as rb_y_gt:
    for line in rb_y_gt.readlines():
        j += 1
        if j <= time_len:
            y_gt.append(float(line))
rb_y_gt.close()

l = 0
with open('theta_gt.dat') as rb_theta_gt:
    for line in rb_theta_gt.readlines():
        l += 1
        if l <= time_len:
            theta_gt.append(float(line))
rb_theta_gt.close()

plt.plot(x_gt, y_gt, c='darkcyan', label='Groundtruth')
# print('Robot pose groundtruth plotting finished!')

with open('landmark_gt.dat') as lm_gt:
    for line in lm_gt.readlines():
        temp = line.split()
        lm_x_gt.append(float(temp[0]))
        lm_y_gt.append(float(temp[1]))
plt.scatter(lm_x_gt, lm_y_gt, c='darkcyan', marker='o', s=4)
lm_gt.close()
# print('Landmark position groundtruth plotting finished!')

plt.title('Comparison of Results')
plt.xlabel('x[m]')
plt.ylabel('y[m]')
plt.legend(loc='upper right')
plt.savefig('comparison.png')
plt.clf()
print('First image saved!')
# plt.show()


plt.figure(2)
err_x = [x_gt[i] - x[i] for i in range(0, len(x))]
err_y = [y_gt[i] - y[i] for i in range(0, len(y))]
err_theta = [determine_angle(theta_gt[i] - theta[i]) for i in range(0, len(theta))]
timestamp = range(time_len)
plt.plot(timestamp, err_x, label='x position', c='peru')
plt.plot(timestamp, err_y, label='y position', c='darkcyan')
plt.plot(timestamp, err_theta, label='orientation', c='olivedrab')
plt.title("Robot Pose Error for each Time Stamp")
plt.legend()
plt.xlabel("Time Stamp")
plt.ylabel("Error [m] or [rad]")
plt.savefig('robot_pos_err.png')
plt.clf()
print('Second image saved!')

plt.figure(3)
err_lm_x = [lm_x_gt[i] - lm_x[i] for i in range(17)]
err_lm_y = [lm_y_gt[i] - lm_y[i] for i in range(17)]
lm_num = range(17)
plt.plot(lm_num, err_lm_x, label='x position', c='peru')
plt.plot(lm_num, err_lm_y, label='y position', c='darkcyan')
plt.title("Landmark Error")
plt.legend()
plt.xlabel("Landmark ID")
plt.ylabel("Error [m]")
plt.savefig('landmark_err.png')
plt.clf()
plt.close()
print('Third image saved!')


fra = 2
counter = 0
plt.figure()
for fra in range(2, 1986, 10):
    x2plot = x[:fra]
    y2plot = y[:fra]
    x_gt2plot = x_gt[:fra]
    y_gt2plot = y_gt[:fra]
    plt.plot(x2plot, y2plot, c='peru', label='SLAM')
    plt.plot(x_gt2plot, y_gt2plot, c='darkcyan', label='Groundtruth')
    plt.legend(loc='upper right')
    plt.savefig(save_path + str(counter + 1) + ".png")
    plt.clf()
    counter += 1
print("All images saved!")


