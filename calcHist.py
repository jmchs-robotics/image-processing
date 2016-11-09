import cv2
import numpy as np

img = cv2.imread('pix/stronghold_green.png')
cv2.imshow('Input',img)

# convert to hsv
# OpenCV uses ranges: hue 0-180, sat 0-255, val 0-255
# gimp uses ranges H = 0-360 degrees, S = 0-100% and V = 0-100%
img = cv2.cvtColor( img, cv2.COLOR_BGR2HSV)

# create a mask for the area where the target is
# 150,430 to 250,600
width, height = img.shape[0], img.shape[1]
mask = np.zeros(( width, height), np.uint8)
cv2.rectangle( mask, ( 430,150), ( 600,250), 255, thickness=-1)
#cv2.circle( mask, (width/2,height/2),280,1,thickness=-1)
#img = cv2.bitwise_and(img, img, mask=mask)

cv2.imshow( 'Masked input', cv2.bitwise_and(img, img, mask=mask))
    
cv2.waitKey(0)
cv2.destroyAllWindows()

# mask = None

# print histogram for all 3 color components at once
histrL = []

for i in range(3):
    histrL.append( cv2.calcHist([img],[i], mask,[256],[0,256]))

print 'i, hue, sat, val'
for i in range(256):
    print "%4d, %6d, %6d, %6d" % ( i, histrL[0][i], histrL[1][i], histrL[2][i])


quit()

# find histogram for one color at a time
color = ('hue') #,'g','r')
for channel,col in enumerate(color):
    histr = cv2.calcHist([img],[channel],None,[256],[0,256])
    print col
    for i in range(256):
        print "%d" % histr[i] #, hist


