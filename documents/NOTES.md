
Lenses
======

KOWA LM4NCL or LM5NCL
<https://www.kowa-lenses.com/en/applications/machine-vision/269/lm5ncl>

Theia Technologies MY110M
<https://www.theiatech.com/sy110-ultra-wide/>

https://www.edmundoptics.com/
https://www.vision-dimension.com/
https://industry.ricoh.com/

PINS
====

Rouge: power IN
Noir: power GND
Bleu: trigger GND
Marron: trigger IN

TODO
====

* Improve recording framerate
  * grayscale instead of RGB?
  * encode with ffmpeg instead of writing uncompressed images?
* Use CUDA on the whole SLAM pipeline and for recording
  * FFMPEG/CUDA: https://stackoverflow.com/questions/44510765/gpu-accelerated-video-processing-with-ffmpeg
  * For bundle adjustment: https://github.com/cbalint13/pba
  * Understand why number of keypoints is incorrect when using cv::cuda::ORB
* Improve temporal matching and validate SLAM pipeline
* Work on undistorted coordinates or rectify the image
* Hardware synchronization of the cameras

