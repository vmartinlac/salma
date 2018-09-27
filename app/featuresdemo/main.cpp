#include "FeatureDetector.h"
#include "opencv2/highgui.hpp"
#include <iostream>

int main(int num_args, char** args)
{
    if( num_args != 2 ) exit(1);

    cv::Mat im = cv::imread(args[1]);

    if( im.data == nullptr ) exit(1);

    FeatureDetector o;
    std::vector<cv::KeyPoint> kpts;
    cv::Mat descr;
    o.run(im, kpts, descr);

    std::cout << "Number of keypoints: " << kpts.size() << std::endl;

    cv::drawKeypoints(im, kpts, im);
    cv::imshow("image", im);
    cv::waitKey(0);

    return 0;
}

