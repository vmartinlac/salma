#include "target.h"
#include <iostream>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include "utils.h"

namespace target {

/*
Adapt a std::vector<cv::KeyPoint> for usage by nanoflann.
*/

class KeyPointListAdapter
{
public:

    KeyPointListAdapter(const std::vector<cv::KeyPoint>* list) :
        m_key_points(list)
    {
        ;
    }

    inline size_t kdtree_get_point_count() const
    {
        return m_key_points->size();
    }

    inline float kdtree_get_pt(size_t index, int dim) const
    {
        cv::Vec2f pt = m_key_points->operator[](index).pt;
        return pt[dim];
    }

    template<typename BBOX>
    inline bool kdtree_get_bbox(BBOX& bb) const
    {
        return false;
    }

protected:

    const std::vector<cv::KeyPoint>* m_key_points;
};

bool Detector::filter_keypoint(const cv::KeyPoint& kp)
{
    cv::Mat& image = m_thresh;

    if( image.type() != CV_8U) throw std::runtime_error("bad type of image");

    const float radius = 15.0;

    const float dalpha = 1.5 / radius;

    int symm = 0;
    int antisymm = 0;
    int white = 0;
    int black = 0;

    for(float alpha = 0.0; alpha<M_PI; alpha += dalpha)
    {
        const float dx = radius*cos(alpha);
        const float dy = radius*sin(alpha);

        const cv::Point P1( kp.pt.x + dx, kp.pt.y + dy );
        const cv::Point P2( kp.pt.x - dx, kp.pt.y - dy );

        const uint8_t val1 = image.at<uint8_t>(P1);
        const uint8_t val2 = image.at<uint8_t>(P2);

        if( val1 == val2 )
        {
            symm++;
        }
        else
        {
            antisymm++;
        }

        if( val1 ) white++; else black++;
        if( val2 ) white++; else black++;
    }

    const float ratio1 = float(symm) / float(antisymm + symm);
    const float ratio2 = float(white) / float(white + black);

    return ( ratio1 > 0.7 && 0.35 < ratio2 && ratio2 < 0.65 );
}

void Detector::filter_keypoints()
{
    std::vector<cv::KeyPoint>::iterator it = m_keypoints.begin();

    while(it != m_keypoints.end())
    {
        if(filter_keypoint(*it))
        {
            it++;
        }
        else
        {
            *it = std::move(m_keypoints.back());
            m_keypoints.pop_back();
        }
    }

    std::cout << "Number of key points: " << m_keypoints.size() << std::endl;
}

/*
struct Target
{
    cv::Point2f origin;
    cv::Vec2f directions[3];
    float score;
};
*/


void Detector::run(const cv::Mat& image)
{
    m_image = &image;

    std::cout << "Converting to greyscale..." << std::endl;
    cv::cvtColor(image, m_greyscale, CV_BGR2GRAY);

    std::cout << "Thresholding..." << std::endl;
    cv::threshold(m_greyscale, m_thresh, 100, 255, cv::THRESH_BINARY_INV);

    std::cout << "Detecting Shi-Tomasi key points..." << std::endl;
    cv::Ptr<cv::GFTTDetector> detector = cv::GFTTDetector::create();

    detector->setMinDistance(30.0);
    detector->setMaxFeatures(700);

    m_keypoints.clear();
    detector->detect(image, m_keypoints);

    std::cout << "Filtering key points..." << std::endl;
    filter_keypoints();

    std::cout << "Constructing kd-tree..." << std::endl;
    KeyPointListAdapter adaptor(&m_keypoints);
    KDTree kdtree(2, adaptor);
    kdtree.buildIndex();
    m_kdtree = &kdtree;

    std::cout << "Detecting target..." << std::endl;
    detect_target();

    cv::Mat output;
    cv::drawKeypoints(image, m_keypoints, output);
    ui::imshow(output);
}

void Detector::detect_target()
{
    bool go_on = false;

    while(go_on)
    {
        //size_t idx = rand() % m_keypoints.size();

        ;
    }
}

}

