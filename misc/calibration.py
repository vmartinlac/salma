import cv2
import numpy
import pylab
import os
import sys

if len(sys.argv) != 2:
    print("Please specify a directory where to find object <-> image correspondances.")
    exit(1)

path = sys.argv[1]

object_points = []
image_points = []

for name in os.listdir(path):
    data = numpy.loadtxt(os.path.join(path, name))
    object_points.append( numpy.asarray( data[:,:3], dtype=numpy.float32) )
    image_points.append( numpy.asarray( data[:,3:], dtype=numpy.float32) )

err, K, dist, R, T = cv2.calibrateCamera(
    object_points,
    image_points,
    (1292, 964),
    None,
    None)

print("Reprojection error:")
print(err)
print()

print("Calibration matrix:")
numpy.savetxt(sys.stdout, K)
print()

print("Distortion coefficients:")
numpy.savetxt(sys.stdout, dist)
print()

