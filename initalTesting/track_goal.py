#!/usr/bin/python
import cv2
import numpy as np

# read file into image
file = "./pix/stronghold_green.png"
img = cv2.imread(file)

imgH, imgW = img.shape[:2]

# cv2.imshow('image',img)

gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

# define range of blue color in HSV (Blue,green,red)
#lower = np.array([110,50,50])
#upper = np.array([130,255,255])

# define range of green color in HSV (Blue,green,red)
lower = np.array([60,50,50])
upper = np.array([93,255,255])

# Threshold the HSV image to get only green colors
mask = cv2.inRange(hsv, lower, upper)
cv2.imshow('mask',mask)

# find edges of green shape
edges = cv2.Canny(mask,30,125,apertureSize = 3)
lines = cv2.HoughLinesP(edges, 1, np.pi/32, 6, None, 30, 10);
for line in lines[0]:
    pt1 = (line[0],line[1])
    pt2 = (line[2],line[3])
    cv2.line(img, pt1, pt2, (0,0,255), 2)
cv2.imshow('Hough Lines',img)
cv2.imshow('edges',edges)

# Bitwise-AND mask and original image
#res = cv2.bitwise_and(img,img, mask= mask)
#cv2.imshow('res',res)

# find contours in the mask image, which is black and white
contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
# find largest contour, by area
c = max(contours, key = cv2.contourArea)
# find bounding rectangle, ( center (x,y), (width, height), angle of rotation )
r = cv2.minAreaRect(c)
# get four corners of bounding box
box = np.int0(cv2.cv.BoxPoints(r))

# draw the bounding box around the U
cv2.drawContours(img, [box], -1, (0, 255, 0), 2)
# draw a circle in the center of the U
cv2.circle(img, (int(r[0][0]),int(r[0][1])), 5, (0,0,255))
           
cv2.drawContours(img,[box],-1,(0,0,255),2)
cv2.imshow('contours',img)
cv2.waitKey(0)
cv2.destroyAllWindows()
print( "Four corners")
print(box)
print( "center")
print( r[0])
trackDir = "L"
if( r[0][1] < imgW/2):
    trackDir = "R"
print( "track by moving " + trackDir)
quit()


#edges = cv2.Canny(mask,100,200,apertureSize = 3)
edges = cv2.Canny(mask,30,200,apertureSize = 3)
cv2.imshow('edges',edges)
cv2.waitKey(0)
cv2.destroyAllWindows()
quit()



