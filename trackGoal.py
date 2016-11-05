#!/usr/bin/python
#
# trackGoal.py
#
import cv2
import numpy as np
import time
import pyrealsense as pyrs
import socket
import sys
import os
import re

# set to true to show images
showImages = False
# set to true to print tracking to console
printToConsole = False


print 'Starting...'
#
# start the RealSense camera
#
print 'Setting up RealSense device...'
# pyrs.start() # sets everything to 640x480 and 60 fps
pyrs.start( c_height=1080, c_width=1920, c_fps=30, d_height=480, d_width=640, d_fps=30)
print "\nSleeping 2..."
time.sleep(2)

#
# set ip, port for socket control
#
# default ip set to broadcast address
# ip = '192.168.1.255'
f = os.popen( 'ifconfig |grep "cast"')
a = f.read()
a = re.search( 'cast[:\s](\d+\.\d+\.\d+\.\d+)', a)
ip = a.group(1)

port = 59330

# user-supplied ip and port
if len(sys.argv) >= 2:
    port = sys.argv[1]
    if len(sys.argv) >= 3:
      ip = sys.argv[2]
print( "Writing UDP to %s:%s" % ( ip, port))
# initialize UDP socet
s = socket.socket( socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt( socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

# read file into image
file = "./pix/stronghold_green.png"
#file = "./r.jpg"
#img = cv2.imread(file)

# get this many frames.
framesToGrab = 300
# frame counter increment.  Set to zero to grab frames endlessly (infinite loop), othersise set to 1
frameCounterInc = 1
if frameCounterInc == 1:
  print "Processing %d frames..." % framesToGrab
else:
  print "Processing infinite loop..."

i = 0
startTime = time.time()
while( i < framesToGrab):
    i += frameCounterInc
    
    # read color image from RealSense camera
    imgIn = pyrs.get_colour()
    imgH, imgW = imgIn.shape[:2]
    
    # only process the middle 1/3 of image width, full height
    img = imgIn[ 0:imgH, imgW/3 : imgW*2/3]
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

    trackDir = "L"
    if( r[0][1] < imgW/2):
        trackDir = "R"
    outString = str( int(r[0][0])) + ", " + str( int( r[0][1])) + ", " + trackDir
    if printToConsole: print outString

    # write tracking instructions to socket
    s.sendto( outString, ( ip, port))

print "%d fps" % ( framesToGrab / (time.time() - startTime))

