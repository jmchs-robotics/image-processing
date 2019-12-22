#
# calcHist.py
#
# calcHist.py inFileName [ x1 x2 y1 y2]
# x1 ... y2 define region of interest
#

import cv2
import numpy as np
import sys

inFileName = sys.argv[1]

usingROI = False
if( len( sys.argv) > 2):
    usingROI = True
    x1 = int( sys.argv[ 2])
    x2 = int( sys.argv[ 3])
    y1 = int( sys.argv[ 4])
    y2 = int( sys.argv[ 5])
    if( x1 > x2):
        t = x1
        x1 = x2
        x2 = t
    if( y1 > y2):
        t = y1
        y1 = y2
        y2 = t

img = cv2.imread( inFileName)
cv2.imshow('Input',img)

# create a mask for the area where the target is
# 150,430 to 250,600
width, height = img.shape[0], img.shape[1]
if( usingROI):
    mask = img.copy() # np.zeros(( width, height), np.uint8)
    #cv2.rectangle( mask, ( 430,150), ( 600,250), 255, thickness=-1)
    cv2.rectangle( mask, ( x1,y1), ( x2,y2), ( 0, 0, 255), thickness = 2)
    cv2.imshow( 'Input and Region of Interest', mask) # cv2.bitwise_and(img, img, mask=mask))
    #img2 = img[ 150:250, 430:600 ]
    img2 = img[ y1:y2, x1:x2 ]
    cv2.imshow( 'Region of Interest', img2)
else:
    y1 = 0
    x1 = 0
    y2 = height
    x2 = width

cv2.waitKey(0)
cv2.destroyAllWindows()

# convert to hsv
# OpenCV uses ranges: hue 0-180, sat 0-255, val 0-255
# gimp uses ranges H = 0-360 degrees, S = 0-100% and V = 0-100%
imgHSV = cv2.cvtColor( img[ y1:y2, x1:x2], cv2.COLOR_BGR2HSV)

# print histogram for all 3 color components at once
histrL = []

for i in range(3):
# histrL.append( cv2.calcHist([img],[i], mask,[256],[0,256]))
    histrL.append( cv2.calcHist([imgHSV],[i], None,[256],[0,256]))

print ('i, hue, sat, val')
for i in range(256):
    print ("%4d, %6d, %6d, %6d" % ( i, histrL[0][i], histrL[1][i], histrL[2][i]))


