#!/usr/bin/python3

from cmath import pi
import csv
from time import time
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import scipy.interpolate
import scipy.optimize
import sys
import os
import math

def csv_read(filename, fields):
    data = dict()
    for f in fields:
        data[f] = []
    with open(filename) as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            for f in fields:
                data[f].append(float(row[f]))
    for f in fields:
        data[f] = np.array(data[f])
    return data

def register_path(bag_name,fields_path):
    path = csv_read(bag_name + "/path.csv", fields_path)
    path['time'] = np.delete(path['time'],0)
    path['pos_x'] = np.delete(path['pos_x'],0)
    path['pos_y'] = np.delete(path['pos_y'],0)
    path['yaw'] = np.delete(path['yaw'],0)
    #remove outliers
    for i in range(1,len(path['yaw'])):
        if(6 > abs(path['yaw'][i] - path['yaw'][i-1]) > 0.5):
            path['yaw'][i] = path['yaw'][i-1]
    return path

def register_modified_path(bag_name,fields_path):
    modified_path = csv_read(bag_name + "/modified_path.csv", fields_path)
    #remove outliers
    for i in range(1,len(modified_path['yaw'])):
        if(6 > abs(modified_path['yaw'][i] - modified_path['yaw'][i-1]) > 0.5):
            modified_path['yaw'][i] = modified_path['yaw'][i-1]
    return modified_path

def compute_traj_length(gt):
    length = 0.0
    for i in range(1,len(gt['time'])):
        length += np.linalg.norm([gt['pos_x'][i]-gt['pos_x'][i-1],gt['pos_y'][i]-gt['pos_y'][i-1]])
    return length

def compute_traj_duration(gt):
    return gt['time'][len(gt['time'])-1] - gt['time'][0] 

def main():
    bag_name = sys.argv[1]

    fields_path = ['time', 'pos_x', 'pos_y','yaw','pitch','roll']

    # register and match trajectories with ground truth
    path = register_path(bag_name,fields_path)
    if(os.path.isfile(bag_name + "/modified_path.csv")):
        modified_path = register_modified_path(bag_name,fields_path)

    # display spatial trajectories
    fontsize = 22

    fig, ax1, = plt.subplots(1, 1, constrained_layout=False)
    ax1.plot(path['pos_x'], path['pos_y'], 
             color='forestgreen',label="Scan matcher")
    ax1.plot(modified_path['pos_x'], modified_path['pos_y'], 
             linestyle='--', color='goldenrod',label="Loop closure")
    ax1.plot([modified_path['pos_x'][1],path['pos_x'][1]],
             [modified_path['pos_y'][1],path['pos_y'][1]],
             'o--r',label="Final correction")
    ax1.legend(fontsize=fontsize)
    ax1.set_xlabel('x [m]',fontsize=fontsize)
    ax1.set_ylabel('y [m]',fontsize=fontsize)
    ax1.tick_params(axis='x', labelsize=fontsize)
    ax1.tick_params(axis='y', labelsize=fontsize)
    ax1.grid(True, which='major', color='black', linewidth=0.4)
    ax1.grid(True, which='minor', color='black', linewidth=0.2)

    # evaluation results
    Dx = modified_path['pos_x'][1] - path['pos_x'][1]
    Dy = modified_path['pos_y'][1] - path['pos_y'][1]
    Abs = math.sqrt(Dx**2 + Dy**2)
    
    err_sm = math.sqrt( (path['pos_x'][1] - path['pos_x'][len(path['pos_x'])-1])**2 + 
                        (path['pos_y'][1] - path['pos_y'][len(path['pos_x'])-1])**2 )
    err_lc = math.sqrt( (modified_path['pos_x'][1] - modified_path['pos_x'][len(modified_path['pos_x'])-1])**2 + 
                        (modified_path['pos_y'][1] - modified_path['pos_y'][len(modified_path['pos_x'])-1])**2 )

    print(len(modified_path))

    # display evaluation results
    print("--------------------")
    print("Evaluation results:\n")
    traj_length = compute_traj_length(path)
    traj_duration = compute_traj_duration(path)
    print("Trajectory duration = " + "{:.3f}".format((float)(traj_duration)) + " s")
    print("Trajectory length = " + "{:.3f}".format((float)(traj_length)) + " m\n")
    print("Loop-closure correction : ")
    print("\tDx = " + "{:.3f}".format(Dx) + " m")
    print("\tDy = " + "{:.3f}".format(Dy) + " m\n")
    print("\tabs = " + "{:.3f}".format(Abs) + " m\n")
    print("Approx final APE : ")
    print("\tScan matcher : "+ "{:.3f}".format(err_sm) + " m\n")
    print("\tLoop-closure : "+ "{:.3f}".format(err_lc) + " m\n")

    plt.show()


if __name__ == '__main__':
    main()

