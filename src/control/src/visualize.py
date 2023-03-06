import matplotlib.pyplot as plt
import numpy as np
import time
import math

x=[]
y=[]
cur_x = []
cur_y = []

plot_x = []
plot_y = []

yaw = []
index = []
with open("/home/mz/Documents/zOS/src/SAL/gps/src/path.txt", "r") as f:
    points = [float(line.strip()) for line in f.readlines()]

print(type(points))
for i in range(len(points)):
    if (i%2==0):
        x.append(points[i])
    else:
        y.append(points[i])

for i in range(len(x)-7):
    cumum_sum = 0

    for j in range(5):
        diff= math.atan2(y[i+j+2] - y[i+j+1], x[i+j+2] - x[i+j+1]) - math.atan2(y[i+j+1] - y[i+j], x[i+1+j] - x[i+j])
        cumum_sum+=abs(diff)

    yaw.append(cumum_sum/5)
    index.append(i)

max_speed = 3.0
min_speed = 2.0
cur_yaw = 0.0
speed  = (max_speed - (cur_yaw/0.2) * (max_speed - min_speed))
print("yaw", cur_yaw, "speed", speed)
# with open("/home/minuszero/zOS/src/control/src/tracked_path.txt", "r") as f:
#     points = [float(line.strip()) for line in f.readlines()]

# print(type(points))
# for i in range(len(points)):
#     if (i%2==0):
#         cur_x.append(points[i])
#     else:
#         cur_y.append(points[i])

plt.figure(1) 
plt.bar(index, yaw)
plt.figure(2) 
plt.scatter(x,y)
# plt.scatter(cur_x,cur_y)


# for j in range(len(cur_x)):
#     plot_x.append(cur_x[j])
#     plot_y.append(cur_y[j])
#     plt.scatter(plot_x, plot_y, s=0.5)
    # plt.pause(0.05)

plt.show()



