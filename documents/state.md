
Specifications
==============

Features
--------
Generate which as uniformly distributed and stable (visible for a few successive frames).

Temporal matching
-----------------
Generate as many matches as possible.
Generate as few outliers as possible.
beaucoup de matches.

Alignment
---------

Find the correct pose despite presence of outliers left by temporal matching.

EKF
---

Stereo matching
---------------
Generate as many matches as possible.
Generate as few outliers as possible.

Triangulation
-------------
Triangulate all stereo matches except when parallax is too low.
Estimate covariance of triangulated point wrt to covariance of rig pose.

TO DO
=====

Local Bundle Adjustment
Improve the robustness
    Test/check functions
    Find and understand failure regimes
    Correct failures
UI improvements
    Let the user remove calibration, recording or reconstruction from the UI
    let the user see available cameras
    let the user set configuration from the UI
Dense reconstruction
Improve debug output
    For each reconstruction, generate a set of html pages

Study of regimes of failures
============================

Tests
=====

Which components to test
- computation of mappoint covariance after triangulation
- computation of f and its jacobian
- computation of h and its jacobian

Notes
=====

Interrogation local map
-----------------------

Maintenir la local map en dehors du module EKF?

Le module temporal matcher matches seulement des points de la local map.
Le module EKF diminue la local map si besoin.
Le module triangulation aggrandit la local map.

