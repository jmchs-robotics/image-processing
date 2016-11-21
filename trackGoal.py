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
# set default parameter
#  then parse command line for use parameters
#
inFileName = ''
inDepthFileName = ''
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
f = os.popen( '/sbin/ifconfig |grep "cast"')
a = f.read()
a = re.search( 'cast[:\s](\d+\.\d+\.\d+\.\d+)', a)
try:
    ip = a.group(1)
except:
    ip = '127.0.0.1'
# use axis camera for input
useAxisCam = False
# one less than the number of image frames (for L/R aiming) to one depth map (for distance)
img2depthRatio = 10
# write output files r.jpg and d.jpg of last images, with target areas inscribed
writeSnapshotFiles = False

i = 0
for a in sys.argv:
    if( a == '--images'):
        showImages = True
    elif( a == '--print'):
        printToConsole = True
    elif( a == '--file'):
        inFileName = sys.argv[ i+1]
    elif( a == '--depthFile'):
        inDepthFileName = sys.argv[ i+1]
    elif( a == '--limit'):
        framesToGrab = int( sys.argv[ i+1])
    elif( a == '--port'):
        port = sys.argv[ i+1]
    elif( a == '--ip'):
        ip = sys.argv[ i+1]
    elif( a == '--axis'):
        useAxisCam = True
    elif( a == '--ratio'):
        img2depthRatio = int( sys.argv[ i+1])
    elif( a == '--snapshots'):
        writeSnapshotFiles = True
    i += 1

if len( inFileName) > 0:
    print "Processing input file %s" % ( inFileName)
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

#
# define the color thresholds for isolating the target
#
# OpenCV uses ranges: hue 0-180, sat 0-255, val 0-255
# gimp uses ranges H = 0-360 degrees, S = 0-100% and V = 0-100%
# in Java OpenCV define the thresholds as: Scalar(0,0,80), new Scalar(240, 255, 255)
#

"""
# define range of blue color in HSV
lower = np.array([110,50,50])
upper = np.array([130,255,255])

# define range of green color in HSV
#  for WPI Screensteps Stronghold image online
lower = np.array([60,50,50])
upper = np.array([93,255,255])

# WPI thresholds refined, using gimp and by plotting histograms (Stronghold image)
lower = np.array([ 78, 200, 50])
upper = np.array([ 95, 255, 255])

# range of green color in HSV
#  for Fowkes' old green LEDs and Axis camera
lower = np.array([ 0, 0, 80])
upper = np.array([ 240, 255, 255])
"""
    
# white, HSV
lower = np.array([ 0, 0, 250])
upper = np.array([ 5, 5, 256])

# white and pink, BGR
lower = np.array([ 250, 128, 250])
upper = np.array([ 256, 256, 256])

# select region of interest of map, i.e. where the can should be
#  depth map is always 640w x 480h; FOV is 59deg w x 46deg h
#  see Alex's email to Jon 10/24/16, search on 'isosceles'
#  height of field of view of RealSense at distance d is
#  h = 2 * d * sin( 23deg) / sin( 67deg)
#  can height in pixels is then chp = 25" * 480 * / h
#  can width in pixels cwp = 20" * 640 / ( 2 * d * sin( 29.5deg) / sin( 60.5deg))
# go with 29 pixel width ROI taken from the center;
#  and 29 pixel height centered 15 pixels above center of image.  Images have (0,0) at upper left corner.
# region of numpy array is chosen by [y1:y2, x1:x2].  To extract the region, need y1 < y2 and x1 < x2
y2 = 239
y1 = 210
x1 = 305
x2 = 334

# select only values > nearest can distance minus about 10%
# see http://stackoverflow.com/questions/13869173/numpy-find-elements-within-range
#  and < farthest can distance
#  nearest can is 10' - 3.5' = 6.5' ; 6.5' * 90% = 1783mm
#  farthest can is 20' (assuming RealSense camera is between center of shooting circle and edge)
#  20' * 1.1 = 6705mm
closestCanDepth = 1783
farthestCanDepth = 6705


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
    imgIn = cv2.imread( inFileName) # read the input image from a file
    
if( len( inDepthFileName) > 0): # read the input depth map from a file
    print "Processing depth file %s" % ( inDepthFileName)
    dImgIn = cv2.imread( inDepthFileName) * 8.0 # RealSense vals are 8 times those in the saved image
else:
    dImgIn = None

# initialize UDP socet
s = socket.socket( socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt( socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
print( "Writing UDP to %s:%s" % ( ip, port))


if frameCounterInc == 1:
    print "Processing %d frames..." % ( framesToGrab)
else:
    print "Processing infinite loop..."


# number of pixels 'close enought' to on target
#  when the target is within (+/-) this many pixels of center the UDP and printout
#  will end in 'C' instead of 'L' or 'R'
closeToCenter = 2


# pre-set printout variables, in case there are processing errors
ctr2targetUD = 0 # pixels from center up/down
ctr2targetLR = 0 # pixels from center left/right
r = [(0,0),(-1,-1)] # (( center), ( width, height)) of square around target
trackDir = "C" # which direction to turn


#
# main loop
#
i = 0
img2depthCtr = 0
startTime = time.time()
while( i < framesToGrab):
    #
    # process depth map every img2depthRatio counts of img2depthCtr
    #
    if( img2depthCtr == 0):
        try:
            # get depth map
            if( pyrs):
                dImgIn = pyrs.get_depth()  # each 'pixel' is depth (distance) in mm

            # select pixels in the ROI, i.e. where the can should be in the depth image
            dImg = dImgIn[ y1:y2, x1:x2]

            # select pixels between max and min distance (depth) to cans and then take the median
            canArray = dImg[ np.where( np.logical_and( dImg > closestCanDepth, dImg < farthestCanDepth))]
            canDist = np.average( canArray)

            # show 'image'
            if( showImages or writeSnapshotFiles):
                dImgIn = dImgIn / 8.0
                cv2.rectangle( dImgIn, (x1,y1), (x2,y2), (0, 0, 255), 2)
            if( showImages):
                cv2.imshow( 'raw depth with ROI', dImgIn)

            outString = "Depth-map-based distance = {: 8.1f}".format( canDist)
        except:
            outString = "Error in depth processing for can distance."
            
        # print to console
        if( printToConsole):
            print outString

        # write to UDP
        s.sendto( outString, ( ip, port))
    
    img2depthCtr += 1
    if( img2depthCtr >= img2depthRatio):
        img2depthCtr = 0

    #
    # process image
    #
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
        
        # hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        # test to see if BGR threshold works, red filter on spotlight
        hsv = img
        # Threshold the HSV image to get only green colors
        mask = cv2.inRange(hsv, lower, upper)
        
        # find contours in the mask image, which is black and white
        contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        # find largest contour, by area
        c = max(contours, key = cv2.contourArea)
        # find bounding rectangle, ( center (x,y), (width, height), angle of rotation )
        r = cv2.minAreaRect(c)


        #
        # draw target areas and display
        #
        if( showImages):
            cv2.imshow( 'image', img)
            cv2.imshow( 'mask', mask)
            
        if( showImages or writeSnapshotFiles):
            # get four corners of bounding box
            box = np.int0(cv2.cv.BoxPoints(r))
            # draw the bounding box around the target
            cv2.drawContours(img, [box], -1, (0, 255, 0), 2)
            # draw a circle in the center of the target
            cv2.circle(img, (int(r[0][0]),int(r[0][1])), 5, (0,0,255))
        if( showImages):
            cv2.imshow('contours',img)


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
if( writeSnapshotFiles):
    print "Saving last set of captured images..."
    cv2.imwrite( 'colorWithTargets.jpg', img)
    cv2.imwrite( 'colorMasked.jpg', mask)
    cv2.imwrite( 'depthWithROI.jpg', dImgIn)

if( showImages):
    cv2.waitKey(0)
    cv2.destroyAllWindows()


