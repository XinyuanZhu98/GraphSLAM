import os
import cv2


videowriter = cv2.VideoWriter("evolution_rand.mp4",
                              cv2.VideoWriter_fourcc(*'mpv4'),
                              5, (640, 480))

path = r"C:/Users/Xinyuan Zhu/PycharmProjects/ECE1505_Proj_2000/figure_rand/"
for i in os.listdir(path):
    img = cv2.imread(path + i)
    videowriter.write(img)
videowriter.release()


