#!/usr/bin/python
#
# trackGoal.py
#
import cv2
import numpy as np

# set to true to show images
showImages = False
showImages = True

# read file into image
file = "./pix/stronghold_green.png"
file = "./r.jpg"
img = cv2.imread(file)

imgH, imgW = img.shape[:2]

if showImages: cv2.imshow('image',img)

hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

# define range of blue color in HSV
#lower = np.array([110,50,50])
#upper = np.array([130,255,255])

# define range of green color in HSV 
#  for WPI Screensteps Stronghold image online
lower = np.array([60,50,50])
upper = np.array([93,255,255])

# define range of green color in HSV 
#  for Fowkes' old green LEDs and Axis camera
#  Scalar(0,0,80), new Scalar(240, 255, 255)
lower = np.array([ 0, 0, 80])
upper = np.array([ 240, 255, 255])

# Threshold the HSV image to get only green colors
mask = cv2.inRange(hsv, lower, upper)
if showImages: cv2.imshow('mask',mask)

# find contours in the mask image, which is black and white
contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
# find largest contour, by area
c = max(contours, key = cv2.contourArea)
# find bounding rectangle, ( center (x,y), (width, height), angle of rotation )
r = cv2.minAreaRect(c)
# get four corners of bounding box
box = np.int0(cv2.cv.BoxPoints(r))

# draw the bounding box around the U
if showImages: cv2.drawContours(img, [box], -1, (0, 255, 0), 2)
# draw a circle in the center of the U
if showImages: cv2.circle(img, (int(r[0][0]),int(r[0][1])), 5, (0,0,255))
           
if showImages: cv2.drawContours(img,[box],-1,(0,0,255),2)
if showImages: cv2.imshow('contours',img)
if showImages: cv2.waitKey(0)
if showImages: cv2.destroyAllWindows()
print( "Four corners")
print(box)
print( "center")
print( r[0])
trackDir = "L"
if( r[0][1] < imgW/2):
    trackDir = "R"
print( "track by moving " + trackDir)

