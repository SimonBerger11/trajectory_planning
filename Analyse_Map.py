import math
from pylab import *
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np


def way_hor(y, x_offset,length,res, invers):
    w_ruck = []
    if invers == False:
        for i in range(0,int((length +1)* (1/res))):
            w_ruck.append((x_offset+ i*res,y,0))
    else:
        for i in range(int(length* (1/res)),-1,-1):
            w_ruck.append((x_offset+ i*res,y,0))
    return w_ruck


def way_ver(x,y_offset,length,res, invers):
    w_ruck = []
    if invers == False:
        for i in range(0,int((length +1)* (1/res))):
            w_ruck.append((x,y_offset+(i)*res,0))
    else:
        for i in range(int(length* (1/res)),-1,-1):
            w_ruck.append((x,y_offset+(i)*res,0))
    return w_ruck


def createLines(res):
    way = []
    way.append(way_hor(5, 5, 10, res,False))
    way.append(way_hor(9, 5, 10,res,False))
    way.append(way_hor(14, 5, 10,res,False))

    way.append(way_ver(14, 5, 9,res,False))
    way.append(way_ver(12, 5, 9,res,False))
    way.append(way_ver(9, 5, 9,res,False))
    way.append(way_ver(5, 5, 9,res,False))

    #way.append(way_hor(5, 5, 10,res,True))
    way.append(way_hor(9, 5, 10,res,True))
    way.append(way_hor(14, 5, 10,res,True))

    way.append(way_ver(14, 5, 9,res,True))
    way.append(way_ver(12, 5, 9,res,True))
    #way.append(way_ver(9, 5, 9,res,True))
    way.append(way_ver(5, 5, 9,res,True))

    return way


