import cv2
import numpy
import math
import pylab

img = cv2.imread('../data/photographies_mire/a.jpg')

img = cv2.resize( img, (640, 480) )

img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

_,img = cv2.threshold(img, 128, 255, cv2.THRESH_BINARY_INV)

img = cv2.erode(img, numpy.ones((3,3), dtype=numpy.uint8), iterations=2)

pylab.imshow(img)
pylab.show()
exit()

img2 = cv2.distanceTransform(255-canny, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)

pylab.imshow(0.01*(img[:,:,0]+img[:,:,1]+img[:,:,2])/3.0+0.7*img2)
pylab.show()
exit()

lines = cv2.HoughLines(canny, 20, math.pi/40.0, 900)
lines = lines.reshape( (lines.shape[0], 2) )

for line in lines:
    rho = line[0]
    costheta = math.cos(line[1])
    sintheta = math.sin(line[1])
    length = 50000.0
    x0 = rho*costheta - length*sintheta
    y0 = rho*sintheta + length*costheta
    x1 = rho*costheta + length*sintheta
    y1 = rho*sintheta - length*costheta
    x0 = int(x0)
    y0 = int(y0)
    x1 = int(x1)
    y1 = int(y1)
    cv2.line(img, (x0,y0), (x1,y1), 10, 5)

pylab.imshow(img)
pylab.show()
