# -*- coding: utf-8 -*-
"""
Created on Sat Dec 30 12:07:19 2017

@author: 羅際禎
"""

def kinect2global(a, b):    # a is vertical pixel, b is horizontal
    import numpy as np
    a = a - 292
    b = b - 351
    paraX = np.load("paraX.npy")
    #print(paraX, paraX.shape)
    paraY = np.load("paraY.npy")
    #print(paraY, paraY.shape)
    X = np.matrix([a,b,a**2,b**2,a*b])*paraX
    Y = np.matrix([a,b,a**2,b**2,a*b])*paraY
    X = float(X)
    Y = float(Y)
    X += 120
    #print(X,Y)
    return (X,Y)