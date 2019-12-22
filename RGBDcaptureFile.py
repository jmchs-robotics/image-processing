#!/usr/bin/python
#
# RGBDcaptureFile.py
#
# read color image and depth image from realsense
# save to r.jpg and d.jpg
#

import time
# import matplotlib.pyplot as plt
import pyrealsense2 as pyrs
print 'Starting...'
# pyrs.start()
pyrs.start( c_height=1080, c_width=1920, c_fps=30, d_height=480, d_width=640, d_fps=30)
print 'Sleeping 2...'
time.sleep(2)

# cm = pyrs.get_colour()
# plt.imshow(cm)
# plt.show()

print "Depth scale = %25.22f" % pyrs.get_depth_scale()

import cv2
import numpy as np
import time

cnt = 0
last = time.time()
first = time.time()
smoothing = 0.9;
fps_smooth = 30

print "Capturing 30 sets of images..."
setSize = 3
while cnt < 30:

    cnt += 1
    if (cnt % 10) == 0:
        now = time.time()
        dt = now - last
        fps = 10/dt
        fps_smooth = (fps_smooth * smoothing) + (fps * (1.0-smoothing))
        last = now

    c = pyrs.get_colour()
    d = pyrs.get_depth()
    
    d = d >> 3
#    irmap = pyrs.get_ir()

    # print c.shape, d.shape, irmap.shape
    # d = cv2.applyColorMap(d.astype(np.uint8), cv2.COLORMAP_RAINBOW)

    # cd = np.concatenate((c,d), axis=1)

    # cv2.putText(cd, str(fps_smooth)[:4], (0,50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,0))

    # cv2.imshow('', cd)
    # if cv2.waitKey(1) & 0xFF == ord('q'):
        # break

# calculate fps
# print "%f fps" % (( cnt / ( time.time() - first)) * setSize)
# min, max
# print "IR min %d, max %d" % ( irmap.min(), float( irmap.max()))
# save the last image set
print "Saving last set of captured images..."
cv2.imwrite( 'r.jpg', c)
cv2.imwrite( 'd.jpg', d)
#cv2.imwrite( 'irmap.png', irmap)

