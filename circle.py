# import the necessary packages
import numpy as np
#import argparse
import cv2
from timer import Timer
timer = Timer()
start = timer.tic()

# load the image, clone it for output, and then convert it to grayscale
image = cv2.imread("2balls.png")
img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
img_hsv = cv2.GaussianBlur(img_hsv, (5,5), 0)
output = image.copy()

cv2.imshow("source",image)
#cv2.imshow("hsv",img_hsv)
cv2.waitKey(0)

'''
# histogram equalization:
img_eq = img_hsv.copy()
img_eq[0,0,:] = cv2.equalizeHist(img_eq[0,0,:])
img_output = cv2.cvtColor(img_eq, cv2.COLOR_HSV2BGR)
cv2.imwrite('eq.jpg',img_output)
# CLAHE (contrast limited adaptive histogram equaliztion)
cla = img_hsv.copy()
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(10,10))
cla[:,:,:] = clahe.apply(cla[:,:,:])
img_output = cv2.cvtColor(cla, cv2.COLOR_HSV2BGR)
cv2.imwrite('clahe.jpg',img_output)
'''

# reference: https://stackoverflow.com/questions/26218280/thresholding-rgb-image-in-opencv
green_low = (55-25, 50, 50)
green_high = (55+25, 200, 200)
image = cv2.inRange(img_hsv, green_low, green_high) 
cv2.imshow("filtered",image)
cv2.waitKey(0)
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
image = cv2.erode(image, kernel, iterations=1)
image = cv2.dilate(image, kernel, iterations=1)
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
image = cv2.erode(image, kernel, iterations=1)
image = cv2.dilate(image, kernel, iterations=1)
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11,11))
image = cv2.erode(image, kernel, iterations=1)
image = cv2.dilate(image, kernel, iterations=1)


#image = cv2.GaussianBlur(image,(3,3),0)
#cv2.Canny(image, 30, 70, image)

cv2.imshow("preprocessed",image)
cv2.waitKey(0)


# detect circles in the image
circles = cv2.HoughCircles(image, cv2.HOUGH_GRADIENT, 10,40,10,32,30) # dp, mindist, ??,??,minradius
# reference: https://docs.opencv.org/2.4/modules/imgproc/doc/feature_detection.html#houghcircles

# convert the (x, y) coordinates and radius of the circles to integers
circles = np.round(circles[0, :]).astype("int")

# loop over the (x, y) coordinates and radius of the circles
for (x, y, r) in circles:
	if r < 20:
		# draw the circle in the output image, then draw a rectangle
		# corresponding to the center of the circle
		cv2.circle(output, (x, y), r, (0, 255, 0), 4)
		cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

cv2.imshow("output",output)
cv2.waitKey(0)


timer.toc()
dt = timer.total_time

print("time elasped = "+str(dt))
