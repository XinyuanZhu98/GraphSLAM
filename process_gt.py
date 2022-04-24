import math

data_old = []
with open('Robot1_Groundtruth.dat') as f_old:
    for line in f_old.readlines():
        temp = line.split()
        data_old.append(temp)
f_old.close()

f_new = open('new_gt.txt', 'w')
data_new = data_old.copy()
for i in range(len(data_new)):
    data_new[i][0] = str(float(data_old[i][0]) - 1248272276.038)
    string = " ".join([str(ele) for ele in data_new[i]])
    f_new.write(string + '\n')
f_new.close()