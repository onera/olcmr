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

### Functions for trajectories relative pose optimisation and same time interpolation
def rotate(data, a):
    Ca = np.cos(a)
    Sa = np.sin(a)
    res = dict()
    res['time'] = data['time']
    res['pos_x'] = Ca*data['pos_x'] - Sa*data['pos_y']
    res['pos_y'] = Sa*data['pos_x'] + Ca*data['pos_y']
    res['yaw'] = np.mod(data['yaw']+a,2*pi)
    return res

def translate(data, x, y):
    res = dict()
    res['time'] = data['time']
    res['pos_x'] = data['pos_x'] + x
    res['pos_y'] = data['pos_y'] + y
    res['yaw'] = data['yaw']
    return res

def synchronize(master,slave):
    res = dict()
    diff = min( master['time']) - min(slave['time'])
    for i in range(len(slave['time'])) :
        slave['time'][i] += diff
    res['pos_x'] = scipy.interpolate.griddata(slave['time'], slave['pos_x'], master['time'])
    res['pos_y'] = scipy.interpolate.griddata(slave['time'], slave['pos_y'], master['time'])
    res['yaw'] = scipy.interpolate.griddata(slave['time'], slave['yaw'], master['time'])
    res['time'] =  master['time']
    return res

def pos_error(gt,data):
    err = []
    mean_err_tot = 0
    for i in range(len(gt['time'])):
        current_err = np.sqrt( (gt['pos_x'][i] - data['pos_x'][i])**2 + (gt['pos_y'][i] - data['pos_y'][i])**2 )
        if( current_err >= 0):
            err.append(current_err)
            mean_err_tot += current_err
        else :
            err.append(err[len(err)-1])
            mean_err_tot += err[len(err)-1]
    mean_err_tot = mean_err_tot/len(err)
    return [err, mean_err_tot]

def angle_error(gt,data):
    err = []
    mean_err_tot = 0
    for i in range(len(gt['time'])):
        current_err = abs(gt['yaw'][i] - data['yaw'][i])
        if( current_err >= 0 ):
            err.append(current_err)
            mean_err_tot += current_err
        else :
            err.append(err[len(err)-1])
            mean_err_tot += err[len(err)-1]
    mean_err_tot = mean_err_tot/len(err)
    for i in range(1,len(err)):
        if(err[i] > 0.5):
            err[i] = err[i-1]
    return [err, mean_err_tot]

def smooth_data(data,n):
    smooth = []
    # for i in range(n//2) :
    #     smooth.append(data[i])
    for i in range(n//2,len(data)-n//2):
        mean = 0
        for j in range(n//2):
            mean += data[i-n//2+j] + data[i+j]
        mean = mean/(n//2*2)
        smooth.append(mean)
    # for i in range(n//2) :
    #     smooth.append(data[len(data)-i-1])   
    return smooth 

def traj_error(a,gt,data):
    candidate = rotate(data,a)
    [err,mean_err_tot] = pos_error(gt,candidate)
    return mean_err_tot

### Functions to compute evaluation metrics
def compute_rpe(gt,data):
    RPE_T = 0.0
    RPE_A = 0.0
    delta = len(gt['time'])-1
    for n in range(1,delta) :
        Q = [np.eye(3)]
        P = [np.eye(3)]
        skip = 0
        RMSE = 0.0
        MYE = 0.0
        for i in range(1,len(gt['time']),n) :
            Q.append( np.mat([
                [np.cos(gt['yaw'][i]),-np.sin(gt['yaw'][i]),gt['pos_x'][i]],
                [np.sin(gt['yaw'][i]), np.cos(gt['yaw'][i]),gt['pos_y'][i]],
                [0, 0, 1]]))
            P.append( np.mat([
                [np.cos(data['yaw'][i]),-np.sin(data['yaw'][i]),data['pos_x'][i]],
                [np.sin(data['yaw'][i]), np.cos(data['yaw'][i]),data['pos_y'][i]],
                [ 0, 0, 1]]))
            if(i>0):
                E = np.linalg.inv( np.linalg.inv(Q[i//n-1]) @ Q[i//n] ) @ np.linalg.inv( np.linalg.inv(P[i//n-1]) @ P[i//n] )
                transE = np.linalg.norm([E[0,2],E[1,2]])
                angE = np.arccos((np.trace(E)-1)/2)
                if(not np.isnan(transE) and not np.isnan(angE)):
                    RMSE += (transE)**2
                    MYE += angE
                else:
                    skip += 1
        RMSE = np.sqrt(1/(len(gt['time'])//n-skip)*RMSE)
        MYE = 1/(len(gt['time'])//n-skip)*MYE
        RPE_T += RMSE
        RPE_A += MYE
    RPE_T = 1/delta*RPE_T
    RPE_A = 1/delta*RPE_A
    return RPE_T, RPE_A

def compute_ate(gt,data):
    RMSE = 0.0
    skip = 0
    for i in range(1,len(gt['time'])):
        if(np.isnan(data['yaw'][i])):
            data['yaw'][i] = data['yaw'][i-1]
        Q = np.mat([
            [np.cos(gt['yaw'][i]),-np.sin(gt['yaw'][i]),gt['pos_x'][i]],
            [np.sin(gt['yaw'][i]), np.cos(gt['yaw'][i]),gt['pos_y'][i]],
            [0, 0, 1]])
        P = np.mat([
            [np.cos(data['yaw'][i]),-np.sin(data['yaw'][i]),data['pos_x'][i]],
            [np.sin(data['yaw'][i]), np.cos(data['yaw'][i]),data['pos_y'][i]],
            [ 0, 0, 1]])
        F = np.linalg.inv(Q) @ P
        transF = np.linalg.norm([F[0,2],F[1,2]])
        if(not np.isnan(transF)):
            RMSE += (transF)**2
        else:
            skip += 1
    ATE = np.sqrt(1/(len(gt['time'])-skip)*RMSE)
    return ATE

def compute_traj_length(gt):
    length = 0.0
    for i in range(1,len(gt['time'])):
        length += np.linalg.norm([gt['pos_x'][i]-gt['pos_x'][i-1],gt['pos_y'][i]-gt['pos_y'][i-1]])
    return length

def compute_traj_duration(gt):
    return gt['time'][len(gt['time'])-1] - gt['time'][0] 

### Convert a quaternion into euler angles (roll, pitch, yaw)
def euler_from_quaternion(x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z # in radians

### register, interpolate and optimise the trajectories
def register_gt(bag_name,fields_gt):
    gt_csv = csv_read(bag_name + "/"+bag_name+".csv", fields_gt)
    gt = dict()
    gt['time'] = []
    gt['pos_x'] = gt_csv['x']
    gt['pos_y'] = gt_csv['y']
    gt['yaw'] = []
    for i in range(len(gt_csv['#sec'])):
        gt['time'].append(float(gt_csv['#sec'][i])+int(gt_csv['nsec'][i]/1000000)*1e-3)
        gt['yaw'].append(np.mod(euler_from_quaternion(
                gt_csv['qx'][i],
                gt_csv['qy'][i],
                gt_csv['qz'][i],
                gt_csv['qw'][i])[2],2*pi))
    gt = translate(gt,-gt['pos_x'][0],-gt['pos_y'][0])
    return gt

def register_path(bag_name,fields_path,gt):
    path = csv_read(bag_name + "/path.csv", fields_path)
    path['time'] = np.delete(path['time'],0)
    path['pos_x'] = np.delete(path['pos_x'],0)
    path['pos_y'] = np.delete(path['pos_y'],0)
    path['yaw'] = np.delete(path['yaw'],0)
    path = synchronize(gt,path)
    a = scipy.optimize.fmin(traj_error,0,args=(gt,path))
    path = rotate(path,a)
    #remove outliers
    for i in range(1,len(path['yaw'])):
        if(6 > abs(path['yaw'][i] - path['yaw'][i-1]) > 0.5):
            path['yaw'][i] = path['yaw'][i-1]
    return path

def register_modified_path(bag_name,fields_path,gt):
    modified_path = csv_read(bag_name + "/modified_path.csv", fields_path)
    modified_path = synchronize(gt,modified_path)
    modified_path = translate(modified_path,-modified_path['pos_x'][0],-modified_path['pos_y'][0])
    a = scipy.optimize.fmin(traj_error,0,args=(gt,modified_path))
    modified_path = rotate(modified_path,a)
    #remove outliers
    for i in range(1,len(modified_path['yaw'])):
        if(6 > abs(modified_path['yaw'][i] - modified_path['yaw'][i-1]) > 0.5):
            modified_path['yaw'][i] = modified_path['yaw'][i-1]
    return modified_path


def main():
    bag_name = sys.argv[1]

    fields_path = ['time', 'pos_x', 'pos_y','yaw','pitch','roll']
    fields_gt = ['#sec', 'nsec', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']

    # register and match trajectories with ground truth
    gt = register_gt(bag_name,fields_gt)
    path = register_path(bag_name,fields_path,gt)
    if(os.path.isfile(bag_name + "/modified_path.csv")):
        modified_path = register_modified_path(bag_name,fields_path,gt)

    # display spatial trajectories
    fontsize = 22

    fig, (ax1, ax2) = plt.subplots(1, 2, sharey=True, constrained_layout=False)
    ax1.plot( gt['pos_x'],  gt['pos_y'], '--b',label="Ground truth")
    ax1.plot(path['pos_x'], path['pos_y'], 'g',label="Scan matcher")
    ax1.legend(fontsize=fontsize)
    ax1.set_xlabel('x [m]',fontsize=fontsize)
    ax1.set_ylabel('y [m]',fontsize=fontsize)
    ax1.tick_params(axis='x', labelsize=fontsize)
    ax1.tick_params(axis='y', labelsize=fontsize)
    ax1.minorticks_on()
    ax1.grid(True, which='major', color='black', linewidth=0.4)
    ax1.grid(True, which='minor', color='black', linewidth=0.2)
    
    if(os.path.isfile(bag_name + "/modified_path.csv")):
        ax2.plot( gt['pos_x'],  gt['pos_y'], '--b',label="Ground truth")
        ax2.plot(modified_path['pos_x'], modified_path['pos_y'], 'y',label="Loop closure")
        ax2.legend(fontsize=fontsize)
        ax2.set_xlabel('x [m]',fontsize=fontsize)
        ax2.tick_params(axis='x', labelsize=fontsize)
        ax2.tick_params(axis='y', labelsize=fontsize)
        ax2.minorticks_on()
        ax2.grid(True, which='major', color='black', linewidth=0.4)
        ax2.grid(True, which='minor', color='black', linewidth=0.2)
    
    # display yaw trajectories
    fig2, (ax3, ax4) = plt.subplots(1, 2, sharey=True, constrained_layout=False)
    smooth_factor = 50

    [err,AYE_path] = angle_error(gt,path)
    smooth_err = smooth_data(err,smooth_factor)
    ax3.plot(gt['time'][smooth_factor//2:len(gt['time'])-smooth_factor//2], smooth_err, 'g',label="Scan matcher")
    # mean_err = []
    # for i in range(len(gt['time'])):
    #     mean_err.append(AYE_path)
    # ax3.plot(gt['time'], mean_err, 'b-.',
    #             label="AYE = "+ "{:.3f}".format((float)(AYE_path)) + " rad")
    ax3.legend(fontsize=fontsize)
    ax3.set_xlabel('t [ns]',fontsize=fontsize)
    ax3.set_ylabel('yaw error [rad]',fontsize=fontsize)
    ax3.tick_params(axis='x', labelsize=fontsize)
    ax3.tick_params(axis='y', labelsize=fontsize)
    ax3.minorticks_on()
    ax3.grid(True, which='major', color='black', linewidth=0.4)
    ax3.grid(True, which='minor', color='black', linewidth=0.2)
    
    if(os.path.isfile(bag_name + "/modified_path.csv")):
        [err,AYE_modified_path] = angle_error(gt,modified_path)
        smooth_err = smooth_data(err,smooth_factor)
        ax4.plot(gt['time'][smooth_factor//2:len(gt['time'])-smooth_factor//2], smooth_err, 'y',label="Loop closure")
        ax4.legend(fontsize=fontsize)
        ax4.set_xlabel('t [ns]',fontsize=fontsize)
        ax4.tick_params(axis='x', labelsize=fontsize)
        ax4.tick_params(axis='y', labelsize=fontsize)
        ax4.minorticks_on()
        ax4.grid(True, which='major', color='black', linewidth=0.4)
        ax4.grid(True, which='minor', color='black', linewidth=0.2)

    # display evaluation results
    print("--------------------")
    print("Evaluation results:\n")
    traj_length = compute_traj_length(gt)
    traj_duration = compute_traj_duration(gt)
    print("Trajectory duration = " + "{:.3f}".format((float)(traj_duration)) + " s")
    print("Trajectory length = " + "{:.3f}".format((float)(traj_length)) + " m\n")
    RPE_path,RYE_path = compute_rpe(gt,path)
    ATE_path = compute_ate(gt,path)
    RPE_modified_path, RYE_modified_path = compute_rpe(gt,modified_path)
    ATE_modified_path = compute_ate(gt,modified_path)
    print("RPE scanmatcher = " + "{:.3f}".format((float)(RPE_path)) + " m")
    print("ATE scanmatcher = " + "{:.3f}".format((float)(ATE_path)) + " m")
    print("RYE scanmatcher = " + "{:.3f}".format((float)(RYE_path)) + " rad")
    print("AYE scanmatcher = " + "{:.3f}".format((float)(AYE_path)) + " rad\n")
    print("RPE loop closure = " + "{:.3f}".format((float)(RPE_modified_path)) + " m")
    print("ATE loop closure = " + "{:.3f}".format((float)(ATE_modified_path)) + " m")
    print("RYE loop closure = " + "{:.3f}".format((float)(RYE_modified_path)) + " rad")
    print("AYE loop closure = " + "{:.3f}".format((float)(AYE_modified_path)) + " rad")

    plt.show()


if __name__ == '__main__':
    main()

