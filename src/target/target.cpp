#include <algorithm>
#include <iostream>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <nanoflann.hpp>
#include "target.h"
#include "utils.h"

namespace target {

bool Detector::filter_line(const cv::Point2f& A, const cv::Point2f& B)
{
    const double l = 10.0;
    const double alpha = 0.1;
    const double beta = 0.1;

    //////

    int white_left = 0;
    int white_right = 0;
    int black_left = 0;
    int black_right = 0;

    cv::Vec2f AB = B - A;
    const double norm_AB = cv::norm(AB);

    cv::Vec2f t = AB / norm_AB;
    cv::Vec2f n = cv::Vec2f(-t(1), t(0));

    for(double s=0.0; s<=norm_AB; s+=1.0)
    {
        cv::Point2f M = cv::Vec2f(A) + s*t;

        cv::Point2f P_left = cv::Vec2f(M) - l*n;
        cv::Point2f P_right = cv::Vec2f(M) + l*n;

        if( m_thresh.at<uint8_t>(P_left) ) white_left++;
        else black_left++;

        if( m_thresh.at<uint8_t>(P_right) ) white_right++;
        else black_right++;
    }

    const int total = white_left + black_left + white_right + black_right;
    const float white_ratio = float(white_left+white_right) / float(total);
    const float white_ratio_left = float(white_left) / float(white_left + black_left);
    const float white_ratio_right = float(white_right) / float(white_right + black_right);

    const bool half_are_white = std::fabs(white_ratio - 0.5) < beta;
    const bool left_is_white = ( white_ratio_left > 1.0-alpha && white_ratio_right < alpha );
    const bool right_is_white = ( white_ratio_left < alpha && white_ratio_right > 1.0-alpha );

    return half_are_white && (left_is_white || right_is_white);
}

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

    std::cout << "Constructing kdtree..." << std::endl;
    build_kdtree();

    std::cout << "Detecting target..." << std::endl;
    detect_target();

    cv::Mat output;
    cv::drawKeypoints(image, m_keypoints, output);
    ui::imshow(output);
}

void Detector::detect_target()
{
    size_t idx = rand() % m_keypoints.size();
    for(size_t idx=0; idx<m_keypoints.size(); idx++) {

    std::vector<size_t> neighbors;

    find_k_nearest_neighbors(idx, 20, neighbors);

    //cv::circle(*m_image, m_keypoints[idx].pt, 10, cv::Scalar(0,255,0), -1);

    for(size_t i : neighbors)
    {
        if(filter_line( m_keypoints[idx].pt, m_keypoints[i].pt ))
        {
            //cv::circle(*m_image, m_keypoints[i].pt, 10, cv::Scalar(0,0,255), -1);
            cv::line(*m_image, m_keypoints[idx].pt, m_keypoints[i].pt, cv::Scalar(0,255,0), 5);
        }
    }
    }
}

void Detector::build_kdtree()
{
    m_kpl_adapter.reset(new KeyPointListAdapter(&m_keypoints));
    m_kdtree.reset(new KDTree(2, *m_kpl_adapter));

    m_kdtree->buildIndex();
}

void Detector::find_k_nearest_neighbors(size_t index, size_t k, std::vector<size_t>& neighbors)
{
    float point[2] = { m_keypoints[index].pt.x, m_keypoints[index].pt.y };

    std::vector<float> distances(k+1);

    neighbors.resize(k+1);

    const size_t effective_size = m_kdtree->knnSearch(
        point,
        k+1,
        &neighbors.front(),
        &distances.front());

    if( neighbors.size() != effective_size )
    {
        neighbors.resize(effective_size);
        distances.resize(effective_size);
    }

    size_t i = 0;

    while(i < neighbors.size())
    {
        if(neighbors[i] == index)
        {
            neighbors[i] = neighbors.back();
            distances[i] = distances.back();

            neighbors.pop_back();
            distances.pop_back();
        }
        else
        {
            i++;
        }
    }

    if( neighbors.size() > k ) throw std::runtime_error("error while computing knn");

    std::sort(
        neighbors.begin(),
        neighbors.end(),
        [&distances] (size_t i, size_t j) { return distances[i] < distances[j]; });
}

}
