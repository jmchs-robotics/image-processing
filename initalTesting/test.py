#!/usr/bin/python
import cv2
import numpy as np

# read file into image
file = "/Users/jon/Utilities_Jon/OpenCV/py/pix/StrongholdField160429/one/d-1461968962.356336-513702013.pgm"
img = cv2.imread(file)

cv2.imshow('image',img)


# ln -s -f /usr/local/Cellar/opencv/2.4.12/lib/python2.7/site-packages/cv2.so  /usr/local/lib/python2.7/site-packages/cv2.so
