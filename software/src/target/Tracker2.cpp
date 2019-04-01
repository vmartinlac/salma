#include <random>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <map>
#include "Tracker2.h"

static void find_maxima(const cv::Mat& image, std::vector<cv::Point>& pts)
{
    const float min_distance_to_border = 20.0;

    std::vector<cv::Point> neighbors{
        cv::Point(-1,0),
        cv::Point(1,0),
        cv::Point(0,1),
        cv::Point(0,-1),
        cv::Point(-1,-1),
        cv::Point(1,-1),
        cv::Point(-1,1),
        cv::Point(1,1)
    };

    const int margin = 2;
    const cv::Rect roi(margin, margin, image.cols-2*margin, image.rows-2*margin);

    cv::MatConstIterator_<float> it = image.begin<float>();

    while(it != image.end<float>())
    {
        if( roi.contains(it.pos()) && *it >= min_distance_to_border )
        {
            bool local_maximum = true;

            for(const cv::Point& delta : neighbors)
            {
                const cv::Point pt = it.pos() + delta;

                if( image.at<float>(pt) > *it )
                {
                    local_maximum = false;
                }
            }

            if(local_maximum)
            {
                pts.push_back(it.pos());
            }
        }

        it++;
    }

    std::cout << pts.size() << std::endl;
}

target::Tracker2::Tracker2()
{
}

bool target::Tracker2::track( const cv::Mat& image, bool absolute_pose )
{
    cv::imwrite("debug_output/10_original.png", image);

    std::cout << "Converting to grayscale..." << std::endl;
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::imwrite("debug_output/20_greyscale.png", gray);

    std::cout << "Adaptative threshold..." << std::endl;
    cv::Mat thresh_a;
    cv::threshold(gray, thresh_a, 0, 255, cv::THRESH_OTSU|cv::THRESH_BINARY_INV);
    cv::imwrite("debug_output/30_threshold.png", thresh_a);

    cv::Mat thresh_b(thresh_a.size(), thresh_a.type());;
    std::transform(
        thresh_a.begin<uint8_t>(),
        thresh_a.end<uint8_t>(),
        thresh_b.begin<uint8_t>(),
        [] (uint8_t x) { return (x == 0) ? 255 : 0; } );
    cv::imwrite("debug_output/40_threshold.png", thresh_b);

    std::cout << "Distance transform..." << std::endl;
    cv::Mat dist_a;
    cv::Mat dist_b;
    cv::distanceTransform(thresh_a, dist_a, cv::DIST_L2, 5, CV_32F);
    cv::distanceTransform(thresh_b, dist_b, cv::DIST_L2, 5, CV_32F);
    cv::imwrite("debug_output/50_dist.png", dist_a);
    cv::imwrite("debug_output/60_dist.png", dist_b);

    std::cout << "Detecting local maxima..." << std::endl;
    std::vector<cv::Point> pts_a;
    std::vector<cv::Point> pts_b;
    find_maxima(dist_a, pts_a);
    find_maxima(dist_b, pts_b);

    std::cout << "A..." << std::endl;
    cv::Mat locmax_a(image.size(), CV_8UC1);
    cv::Mat locmax_b(image.size(), CV_8UC1);
    std::fill( locmax_a.begin<uint8_t>(), locmax_a.end<uint8_t>(), uint8_t(0));
    std::fill( locmax_b.begin<uint8_t>(), locmax_b.end<uint8_t>(), uint8_t(0));

    cv::Mat tmp = image.clone();
    for(const cv::Point& pt : pts_a)
    {
        locmax_a.at<uint8_t>(pt) = uint8_t(255);
        cv::circle(tmp, pt, 5, cv::Scalar(0,255,0), -1);
    }

    for(const cv::Point& pt : pts_b)
    {
        locmax_b.at<uint8_t>(pt) = uint8_t(255);
        cv::circle(tmp, pt, 5, cv::Scalar(255,0,0), -1);
    }

    cv::imwrite("debug_output/rien.png", tmp);

    cv::imwrite("debug_output/70_locmax.png", locmax_a);
    cv::imwrite("debug_output/80_locmax.png", locmax_b);

    std::cout << "B..." << std::endl;
    cv::Mat distlocmax_a;
    cv::Mat labels_a;
    cv::distanceTransform(locmax_a, distlocmax_a, labels_a, cv::DIST_L2, 5);
    
    cv::Mat distlocmax_b;
    cv::Mat labels_b;
    cv::distanceTransform(locmax_b, distlocmax_b, labels_b, cv::DIST_L2, 5);

    cv::Mat rien;
    labels_a.convertTo(rien, CV_8U);
    labels_a = rien;

    cv::imwrite("debug_output/90_labels.png", labels_a);
    cv::imwrite("debug_output/100_labels.png", labels_b);
    
    return false;
}

void target::Tracker2::clear()
{
    TrackerBase::clear();
}

