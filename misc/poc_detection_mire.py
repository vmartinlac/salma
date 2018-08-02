import cv2
import numpy
import math
import pylab

img = cv2.imread('../data/photographies_mire/a.jpg')

img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

d = cv2.GFTTDetector.create(minDistance=30, maxCorners=350)

corners = d.detect(img)

img = cv2.drawKeypoints(img, corners, img)

pylab.imshow(img)

pylab.show()

exit()
