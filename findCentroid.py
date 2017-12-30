# -*- coding: utf-8 -*-
"""
Created on Sat Dec 30 12:21:02 2017

@author: 羅際禎
"""

def findCentroid(image):
    circles = []
    tmp, contours, hier = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for i in contours:
        M = cv2.moments(i)
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        circles.append([cx,cy,10])
    return circles