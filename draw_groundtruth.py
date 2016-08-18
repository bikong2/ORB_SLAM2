#!/root/anaconda2/bin python
# coding=utf-8

import sys, os
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d

if __name__ == '__main__':
    if len(sys.argv) == 1: f = open("./groundtruth.txt")
    elif len(sys.argv) == 2: 
        f = open(sys.argv[1])
        print(sys.argv[1])
    elif len(sys.argv) == 3:
        f1 = open(sys.argv[1])
        f2 = open(sys.argv[2])
    else:
        print "usage: python groundtruth.py groundtruth_file"
        exit()

    if len(sys.argv) <= 2:
        print "drawing..."
        x = []
        y = []
        z = []
        for line in f:
            if line[0] == '#': continue
            data = line.split()
            x.append( float(data[1] ) )
            y.append( float(data[2] ) )
            z.append( float(data[3] ) )
        ax = plt.subplot( 111, projection='3d')
        ax.plot(x, y, z)
        plt.show()
    if len(sys.argv) == 3:
        print "comparing..."
        x = []
        y = []
        z = []
        for line in f1:
            if line[0] == '#': continue
            data = line.split()
            x.append( float(data[1] ) )
            y.append( float(data[2] ) )
            z.append( float(data[3] ) )
        ax = plt.subplot( 121, projection='3d')
        ax.plot(x, y, z)

        x = []
        y = []
        z = []
        for line in f2:
            if line[0] == '#': continue
            data = line.split()
            x.append( float(data[1] ) )
            y.append( float(data[2] ) )
            z.append( float(data[3] ) )
        ax = plt.subplot( 122, projection='3d')
        ax.plot(x, y, z, 'r')
        plt.show()

