import numpy as np
# from upperlimb import UPPERLIMB
import os
import csv
data_dir = os.path.join(os.path.abspath(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))), 'data')

def vicon_csv_reader():
    data_path = os.path.join(data_dir, "vicon_data.csv")
    with open(data_path,'r') as f:
        reader = csv.reader(f)
        # data = [row for row in reader]
        first_row = next(reader)
        print(first_row)
        point_pos = first_row[1]
        point_pos = point_pos.replace('[','')
        point_pos = point_pos.replace(']','')
        point_pos = point_pos.replace('\n','')
        point_pos = point_pos.split(' ')
        print(len(point_pos))
        
        while "" in point_pos:
            point_pos.remove("")
        print(len(point_pos))
        # print(np.array(first_row[1]))

def vicon_mean():
    pass




if __name__ =="__main__":
    vicon_csv_reader()