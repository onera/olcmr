#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import linregress
import sys

def compute_traj_duration(gt):
    return gt['time'][len(gt['time'])-1] - gt['time'][0] 

def main():
    file = sys.argv[1]
    cpu = []
    ram = []
    tick = False
    tick_ram = True
    with open(file) as f:
        for line in f:
            try :
                (key, val) = line.split(" ")
            except ValueError:
                continue
            if(tick):
                cpu.append((float)(val.strip()))
                tick = False
            if(tick_ram):
                try:
                    ram.append((float)(val.strip()))
                    tick_ram = False
                except:
                    tick_ram = False
            if(val.strip()=="\"CPU_ALL\""):
                tick = True
            if(val.strip()=="\"ram\""):
                tick_ram = True

    # display total CPU usage

    if(file=="cpu_eval_summit.txt"):
        total_duration = 516.605  #summit dataset
    elif(file=="cpu_eval_newer_college.txt"):
        total_duration = 198.7   #newer college
    else :
        #print("Please register trajectory duration here, default total duration set to 100 s")
        #total_duration = 114.3	#kitti_06
        total_duration = 287.54	#kitti_05

    mean = np.mean(cpu)
    std = np.std(cpu)
    max = np.max(cpu)
    max_ram = np.max(ram)

    window_size = 20
    cpu_smooth = []
    step = total_duration/len(cpu)
    
    for i in range(window_size//2,len(cpu)-window_size//2):
        cpu_smooth.append(np.mean(cpu[i-window_size//2:i+window_size//2]))

    time = []
    for i in range(len(cpu_smooth)):
        time.append(window_size//2*step+i*step)

    time_ram = []
    for i in range(len(ram)):
        time_ram.append(i*step)

    line_ram = []
    r = linregress(time_ram, ram)
    for t in time_ram :
        line_ram.append(r.intercept+r.slope*t)

    print("mean CPU usage = " + "{:.3f}".format(mean) + " %")
    print("std of CPU usage = " + "{:.3f}".format(std) + " %")
    print("max CPU usage = " + "{:.3f}".format(max) + " %")
    print("max RAM usage = " + "{:.3f}".format(max_ram) + " %")
    print("max RAM time estimated = {:.3f}".format((100-r.intercept)/r.slope)+" s")

    fontsize = 25

    fig, (ax1,ax2) = plt.subplots(2, 1, constrained_layout=False, sharex=True)
    ax1.plot(time, np.ones(len(cpu_smooth))*mean ,'-.',label="Average CPU usage = " + "{:.3f}".format(mean) + " %")
    ax1.plot(time, cpu_smooth ,'purple',label="CPU usage averaged over "+(str)(window_size)+" samples")
    ax1.set_ylabel('CPU [%]',fontsize=fontsize)
    #ax1.set_ylim([0,100])
    ax1.tick_params(axis='y', labelsize=fontsize)
    ax1.tick_params(axis='x', labelsize=fontsize)
    ax1.grid(True, which='major', color='black', linewidth=0.3)
    ax1.grid(True, which='minor', color='black', linewidth=0.2)

    ax2.plot(time_ram, ram ,'grey',label="RAM usage")
    ax2.plot(time_ram, line_ram ,'-.',label="linear fit RAM usage")
    ax2.set_xlabel('time [s]',fontsize=fontsize)
    ax2.set_ylabel('RAM [%]',fontsize=fontsize)
    #ax2.set_ylim([0,100])
    ax2.tick_params(axis='y', labelsize=fontsize)
    ax2.tick_params(axis='x', labelsize=fontsize)
    ax2.grid(True, which='major', color='black', linewidth=0.3)
    ax2.grid(True, which='minor', color='black', linewidth=0.2)
    plt.show()

if __name__ == '__main__':
    main()

