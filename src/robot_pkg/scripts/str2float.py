import csv
import numpy as np


filedir = '/home/lgx/robot_ws/src/robot_pkg/src'
filename = '/home/lgx/robot_ws/src/robot_pkg/src/first_point.csv'
filenew = '/home/lgx/robot_ws/src/robot_pkg/src/first_point_float.csv'
data = []
with open(filename,'r') as f:
    reader = csv.reader(f)
    data = [row for row in reader]

data = np.array(data)

for i in range(data.shape[0]):
    for j in range(data.shape[1]):
        data[i,j] = float(data[i,j])


with open(filenew,'a') as f:
    writer = csv.writer(f)
    writer.writerows(data)


print(data[0,0])