# -*- coding: utf-8 -*-
"""
Created on Wed Dec 20 21:45:52 2017

@author: 羅際禎
"""

import cv2
import numpy as np
from matplotlib import pyplot as plt
import bisect

def imadjust(src, tol=1, vin=[0,255], vout=(0,255)):
    # src : input one-layer image (numpy array)
    # tol : tolerance, from 0 to 100.
    # vin  : src image bounds
    # vout : dst image bounds
    # return : output img

    assert len(src.shape) == 2 ,'Input image should be 2-dims'

    tol = max(0, min(100, tol))

    if tol > 0:
        # Compute in and out limits
        # Histogram
        hist = np.histogram(src,bins=list(range(256)),range=(0,255))[0]

        # Cumulative histogram
        cum = hist.copy()
        for i in range(1, 255): cum[i] = cum[i - 1] + hist[i]

        # Compute bounds
        total = src.shape[0] * src.shape[1]
        low_bound = total * tol / 100
        upp_bound = total * (100 - tol) / 100
        vin[0] = bisect.bisect_left(cum, low_bound)
        vin[1] = bisect.bisect_left(cum, upp_bound)

    # Stretching
    scale = (vout[1] - vout[0]) / (vin[1] - vin[0] + 0.0000001)
    vs = src-vin[0]
    vs[src<vin[0]]=0
    vd = vs*scale+0.5 + vout[0]
    vd[vd>vout[1]] = vout[1]
    dst = vd

    return dst

#kernel = np.ones((3,3),np.float32)/9
img = cv2.imread("2balls.png")
img = cv2.GaussianBlur(img, (3,3), 0)
plt.subplot(121),plt.imshow(img),plt.title('bright')
img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

dark = cv2.imread("dark.png")
dark = cv2.GaussianBlur(dark, (3,3), 0)
plt.subplot(122),plt.imshow(dark),plt.title('dark')
dark = cv2.cvtColor(dark, cv2.COLOR_BGR2HSV)
plt.show()

CLAHE = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(10,10))
green_low = (55-25, 50, 50)
green_high = (55+25, 200, 200)

new = img.copy()
fil1 = cv2.inRange(new, green_low, green_high)
plt.subplot(221),plt.imshow(fil1),plt.title('new')
new[:,:,2] = CLAHE.apply(new[:,:,2])
fil2 = cv2.inRange(new, green_low, green_high)
plt.subplot(223),plt.imshow(fil2),plt.title('clahe new')

new2 = dark.copy()
fil1 = cv2.inRange(new2, green_low, green_high)
plt.subplot(222),plt.imshow(fil1),plt.title('new2')
new2[:,:,2] = CLAHE.apply(new2[:,:,2])
fil2 = cv2.inRange(new2, green_low, green_high)
plt.subplot(224),plt.imshow(fil2),plt.title('clahe new2')
plt.show()

for i in range(3):
    print(i)
    #new[:,:,i] = CLAHE.apply(img[:,:,i])
    #new2[:,:,i] = CLAHE.apply(dark[:,:,i])
    #new[:,:,i] = cv2.equalizeHist(img[:,:,i])
    #new2[:,:,i] = cv2.equalizeHist(dark[:,:,i])

#new[:,:,1] = cv2.equalizeHist(img[:,:,1])
#new2[:,:,1] = cv2.equalizeHist(dark[:,:,1])
#new = cv2.cvtColor(new, cv2.COLOR_HSV2BGR)
#new2 = cv2.cvtColor(new2, cv2.COLOR_HSV2BGR)    
cv2.imwrite("afterhisteq_2balls.jpg",new)
cv2.imwrite("afterhisteq_dark.jpg",new2)