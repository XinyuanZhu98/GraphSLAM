import random

f = open('SLAM_graph_nodes_rand1.dat', 'w')
for i in range(1986):
    f.write(str(10 * random.random()) + '\n')      # x: [0, 10]
    f.write(str(4 * random.random() - 2) + '\n')   # y: [-2, 2]
    f.write(str(6 * random.random() - 3) + '\n')   # theta: [-pi, pi]
    # f.write(str(0) + '\n')
    # f.write(str(0) + '\n')
    # f.write(str(0) + '\n')
for i in range(17):
    f.write(str(10 * random.random()) + '\n')     # lm_x: [0, 10]
    f.write(str(5 * random.random() - 2) + '\n')  # lm_y: [-2, 3]
f.close()
