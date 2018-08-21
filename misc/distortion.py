import cv2
import pylab
import numpy

K = numpy.asarray(
    [[1.085434418781586146e+03,0.000000000000000000e+00,6.252043464245307405e+02],
    [0.000000000000000000e+00,1.079136148982767509e+03,3.443139509297248537e+02],
    [0.000000000000000000e+00,0.000000000000000000e+00,1.000000000000000000e+00]])

dist = numpy.asarray([-8.322383734716308490e-02,1.454580850632900324e+00,-3.150908773569563665e-04,-6.778764863809838385e-03,-4.733991920790191266e+00])

img = cv2.imread('../data/smartphone/calibration_photos/a.png')

img2 = cv2.undistort(img, K, dist)
#mapx, mapy = cv2.initUndistortRectifyMap(K, dist, None, None, img.shape[:2], cv2.CV_32F)
#img2 = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)

pylab.subplot(121)
pylab.imshow(img)
pylab.subplot(122)
pylab.imshow(img2)
pylab.show()

