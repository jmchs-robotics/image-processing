#!/usr/bin/python
#
# trackGoal.py
#
import cv2
import numpy as np
import time
import socket
import sys
import os
import re

#
# parse command line
#
inFileName = ''
frameLimit = -1
# set to True to show images, False to not
showImages = False
# set to True to print tracking to console, False to not
printToConsole = False
# get this many frames.
framesToGrab = -1
# frame counter increment.  Set to zero to grab frames endlessly (infinite loop), othersise set to 1
frameCounterInc = 1
# default port
port = 59330
# default ip - set to local broadcast address
f = os.popen( 'ifconfig |grep "cast"')
a = f.read()
a = re.search( 'cast[:\s](\d+\.\d+\.\d+\.\d+)', a)
ip = a.group(1)
# use axis camera for input
useAxisCam = False


i = 0
for a in sys.argv:
    if( a == '--images'):
        showImages = True
    elif( a == '--print'):
        printToConsole = True
    elif( a == '--file'):
        inFileName = sys.argv[ i+1]
    elif( a == '--limit'):
        framesToGrab = int( sys.argv[ i+1])
    elif( a == '--port'):
        port = sys.argv[ i+1]
    elif( a == '--ip'):
        ip = sys.argv[ i+1]
    elif( a == '--axis'):
        useAxisCam = True
    i += 1

if len( inFileName) > 0:
    print "Processing input file %s" % ( inFileName)
    sys.argv.remove( '--file')
    sys.argv.remove( inFileName)
    framesToGrab = 1
else:
    print "Using RealSense camera for input"

if( framesToGrab < 1):
    framesToGrab = 1
    frameCounterInc = 0

# import pyrealsense if not processing a file
pyrs = None
if( len( inFileName) == 0):
    try:
        import pyrealsense as pyrs
    except:
        pyrs = None


print 'Starting...'

#
# start the RealSense camera or read the input file
#
if pyrs:
    print 'Setting up RealSense device...'
    # pyrs.start() # sets everything to 640x480 and 60 fps
    pyrs.start( c_height=1080, c_width=1920, c_fps=30, d_height=480, d_width=640, d_fps=30)
    print "\nSleeping 2..."
    time.sleep(2)
elif( useAxisCam == True):
    print "Using Axis Camera..."
    vc = cv2.VideoCapture()
    print vc.open("http:axis-00408ca7a2f0.local/mjpg/video.mjpg") # 'http://192.168.1.26/mjpg/video.mjpg')
else:
    # inFileName = "./pix/stronghold_green.png"
    # inFileName = "./r.jpg"
    imgIn = cv2.imread( inFileName)

print( "Writing UDP to %s:%s" % ( ip, port))
# initialize UDP socet
s = socket.socket( socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt( socket.SOL_SOCKET, socket.SO_BROADCAST, 1)


if frameCounterInc == 1:
    print "Processing %d frames..." % framesToGrab
else:
    print "Processing infinite loop..."


# number of pixels 'close enought' to on target
closeToCenter = 2


# pre-set printout variables, in case there are processing errors
ctr2targetUD = 0 # pixels from center up/down
ctr2targetLR = 0 # pixels from center left/right
r = [(0,0),(-1,-1)] # (( center), ( width, height)) of square around target
trackDir = "C" # which direction to turn

i = 0
startTime = time.time()
while( i < framesToGrab):
    try:
        i += frameCounterInc
        
        if pyrs:  # read color image from RealSense camera
            imgIn = pyrs.get_colour()
        elif( useAxisCam == True):  # read color image from Axis camera
            retval, imgIn = vc.read()

        imgH, imgW = imgIn.shape[:2]
        
        # only process the middle 1/3 of image width, full height.
        img = imgIn[ 0:imgH, int(imgW/3.0) : int(imgW*2.0/3.0)]
        imgH, imgW = img.shape[:2]
        
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        
        # OpenCV uses ranges: hue 0-180, sat 0-255, val 0-255
        # gimp uses ranges H = 0-360 degrees, S = 0-100% and V = 0-100%
        
        # define range of blue color in HSV
        #lower = np.array([110,50,50])
        #upper = np.array([130,255,255])
        
        # define range of green color in HSV
        #  for WPI Screensteps Stronghold image online
        lower = np.array([60,50,50])
        upper = np.array([93,255,255])
        
        # refined using gimp and by plotting histograms (Stronghold image)
        lower = np.array([ 78, 200, 50])
        upper = np.array([ 95, 255, 255])
        
        # define range of green color in HSV
        #  for Fowkes' old green LEDs and Axis camera
        #  in Java: Scalar(0,0,80), new Scalar(240, 255, 255)
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
        
        if( showImages):
            cv2.imshow('image',img)
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
        
        # return the distance from the center of the image to the
        #  center of the rectangle, and the dimensions of the rectangle
        ctr2targetUD = ( imgH / 2.0) - r[0][0]
        ctr2targetLR = r[0][1] - ( imgW / 2.0)
        
        trackDir = "C"
        if( ctr2targetLR < - closeToCenter):
            trackDir = "L"
        elif( ctr2targetLR > closeToCenter):
            trackDir = "R"

        # report L_R, up_down, width, height, dir to two decimal places via UDP
        #  target to left/bottom of center return negative positions
        outString = "{: 8.2f}, {: 8.2f}, {: 8.2f}, {: 8.2f}, {:s}".format( ctr2targetLR, ctr2targetUD, r[1][0], r[1][1], trackDir)
    except:
        # if width and/or height is negative then there was no target detected;
        outString = "{: 8.2f}, {: 8.2f}, {: 8.2f}, {: 8.2f}, {:s}".format( ctr2targetLR, ctr2targetUD, -1.0, r[1][1], 'C')
        time.sleep( 1.0 / 60.0)
        print sys.exc_info()[0]

    if printToConsole: print outString

    # write tracking instructions to socket
    s.sendto( outString, ( ip, port))



print "Done. Processing rate was: %d fps" % ( framesToGrab / (time.time() - startTime))
