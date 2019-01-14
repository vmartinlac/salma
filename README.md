Dependencies
============

* Aravis (access to GenICam cameras)
* Eigen3 (linear algebra primitives)
* g2o (bundle adjustment)
* nanoflann (knn search)
* OpenCV (general purpose computer vision)
* OpenMP (shared-memory parallelism)
* openscenegraph (3d visualization)
* Qt5 (user interfaces)
* Sophus (representation of geometric transformations)

Added value
===========

* user interface allowing to calibrate cameras and stereo rigs, make and visualize recordings, run SLAM, visualize reconstructions, etc.
* whole homemade stereo visual odometry engine and dense reconstruction.

Stereo Visual Odometry Pipeline
===============================

The pipeline is devided into 7 parts.

1. Features detection
2. Temporal matching
3. Multiple View PnP
4. Filtering (either Local Bundle Adjustment or Extended Kalman Filter)
5. Stereo matching
6. Triangulation
7. Dense reconstruction

