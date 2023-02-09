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

def register_histogram(file_name,fields):
    histogram = csv_read(file_name,fields)
    return histogram

def main():
    
    file = sys.argv[1]
    fields = ['value', 'class start','class stop']

    # register and match trajectories with ground truth
    histogram = register_histogram(file,fields)

    tot_points = 0
    for val in histogram['value']:
        tot_points += val

    part = tot_points*0.9

    count = 0
    i = 0
    while(count<=part):
        count += histogram['value'][i]
        end = histogram['class stop'][i]
        i += 1

    print("90% error points under "+"{:.3f}".format((float)(end))+" m")


if __name__ == '__main__':
    main()

