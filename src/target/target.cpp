#include <chrono>
#include <cmath>
#include <queue>
#include <algorithm>
#include <iostream>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <nanoflann.hpp>
#include "target.h"
#include "utils.h"

namespace target {

    Detector::Detector() : m_kpl_adapter(&m_points) { }

    bool Detector::run(const cv::Mat& image, cv::Mat& samples)
    {
        std::chrono::time_point<std::chrono::system_clock> t0 = std::chrono::system_clock::now();

        m_image = &image;
        m_debug_image = image.clone();

        std::cout << "Converting to greyscale..." << std::endl;
        cv::cvtColor(image, m_greyscale, CV_BGR2GRAY);

        std::cout << "Thresholding..." << std::endl;
        cv::threshold(m_greyscale, m_thresh, 100, 255, cv::THRESH_BINARY_INV);

        std::cout << "Detecting corners..." << std::endl;
        detect_corners();

        std::cout << "Constructing kdtree..." << std::endl;
        build_kdtree();

        std::cout << "Detecting neighbors..." << std::endl;
        detect_neighbors();

        std::cout << "Enforcing symmetry..." << std::endl;
        symmetrize();

        std::cout << "Computing connected components..." << std::endl;
        compute_connected_components();

        /*
        {
            for(SamplePoint& pt : m_points)
            {
                if( pt.num_neighbors == 4 )
                {
                    cv::Vec2f delta[4];

                    for(int i=0; i<4; i++)
                    {
                        delta[i] = m_points[ pt.neighbors[i] ].keypoint.pt - pt.keypoint.pt;
                    }

                    const double scal1 = delta[0].dot(delta[2]) / ( cv::norm(delta[0])*cv::norm(delta[2]) );
                    const double scal2 = delta[1].dot(delta[3]) / ( cv::norm(delta[1])*cv::norm(delta[3]) );
                    const double threshold = cos(M_PI*0.9);

                    if( scal1 > threshold || scal2 > threshold )
                    {
                        cv::circle(m_debug_image, pt.keypoint.pt, 8, cv::Scalar(0, 255, 0), -1);
                    }
                }
            }
        }
        */

        std::chrono::time_point<std::chrono::system_clock> t1 = std::chrono::system_clock::now();

        const int dt = std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count();

        std::cout << "Computation time: " << dt << " ms" << std::endl;
        std::cout << "Framerate: " << 1.0e3f/float(dt) << " Hz" << std::endl;

        ui::imshow(m_debug_image);

        return false;
    }

    void Detector::build_kdtree()
    {
        m_kdtree.reset(new KDTree(2, m_kpl_adapter));
        m_kdtree->buildIndex();
    }

    void Detector::detect_corners()
    {
        cv::Ptr<cv::GFTTDetector> detector = cv::GFTTDetector::create();

        detector->setMinDistance(30.0);
        detector->setMaxFeatures(700);

        std::vector<cv::KeyPoint> keypoints;
        detector->detect(*m_image, keypoints);

        m_points.clear();

        for( const cv::KeyPoint& kp : keypoints )
        {
            if( filter_keypoint(kp) )
            {
                m_points.emplace_back(kp);
            }
        }
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

    void Detector::detect_neighbors()
    {
        for(int idx=0; idx<m_points.size(); idx++)
        {
            std::vector<int> neighbors;

            find_k_nearest_neighbors(idx, 20, neighbors);

            m_points[idx].num_neighbors = 0;

            int num_BW = 0;
            int num_WB = 0;

            for(int other_idx : neighbors)
            {
                KindOfLine kind;
                if( filter_line( m_points[idx].keypoint.pt, m_points[other_idx].keypoint.pt, kind ) && kind != LINE_NONE )
                {
                    const int neigh_id = m_points[idx].num_neighbors;

                    m_points[idx].num_neighbors++;

                    if( neigh_id < 4 )
                    {
                        m_points[idx].neighbors[neigh_id] = other_idx;
                        m_points[idx].neighbor_types[neigh_id] = kind;
                    }

                    if( kind == LINE_WB ) num_WB++;
                    else num_BW++;
                }
            }

            if( m_points[idx].num_neighbors > 4 )
            {
                m_points[idx].num_neighbors = 0;
            }
            else if( m_points[idx].num_neighbors == 4 )
            {
                if( num_BW != 2 || num_WB != 2 )
                {
                    m_points[idx].num_neighbors = 0;
                    std::cerr << "Some strangeness at " << __FILE__ << ":" << __LINE__ << std::endl;
                }
                else
                {
                    cv::circle(m_debug_image, m_points[idx].keypoint.pt, 5, cv::Scalar(255, 0, 0), -1);
                }
            }
        }
    }

    void Detector::find_k_nearest_neighbors(int index, size_t k, std::vector<int>& neighbors)
    {
        float point[2] = { m_points[index].keypoint.pt.x, m_points[index].keypoint.pt.y };

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
    }

    bool Detector::filter_line(const cv::Point2f& A, const cv::Point2f& B, KindOfLine& kind)
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

        if( half_are_white && (left_is_white || right_is_white) && (left_is_white != right_is_white) )
        {
            kind = (left_is_white) ? LINE_WB : LINE_BW;
            return true;
        }
        else
        {
            kind = LINE_NONE;
            return false;
        }
    }

    void Detector::symmetrize()
    {
        for( int idx = 0; idx<m_points.size(); idx++ )
        {
            SamplePoint& point = m_points[idx];

            int neigh_id = 0;
            
            while( neigh_id < point.num_neighbors )
            {
                const int other_idx = point.neighbors[neigh_id];
                SamplePoint& other_point = m_points[other_idx];

                bool go_on = true;
                for(int other_neigh_id = 0; go_on && other_neigh_id<other_point.num_neighbors; other_neigh_id++)
                {
                    if( other_point.neighbors[other_neigh_id] == idx )
                    {
                        if( other_point.neighbor_types[other_neigh_id] != point.neighbor_types[neigh_id] )
                        {
                            go_on = false;
                        }
                        else
                        {
                            std::cerr << "Some strangeness at " << __FILE__ << ":" << __LINE__ << std::endl;
                        }
                    }
                }

                if(go_on)
                {
                    point.num_neighbors--;
                    point.neighbors[neigh_id] = point.neighbors[point.num_neighbors];
                }
                else
                {
                    neigh_id++;
                }
            }
        }
    }

    void Detector::compute_connected_components()
    {
        // unmark all points.

        for( SamplePoint& pt : m_points )
        {
            pt.connected_component = -1;
        }

        // compute connected components.

        int num_connected_components = 0;
        m_connected_components.clear();

        for(int idx = 0; idx<m_points.size(); idx++)
        {
            if( m_points[idx].num_neighbors == 4 && m_points[idx].connected_component < 0 )
            {
                const int size = find_connected_component(idx, num_connected_components);

                m_connected_components.emplace_back( idx, size );

                num_connected_components++;
            }
        }

        //
        //std::cout << "Sizes of connected components are:" << std::endl;
        //for( ConnectedComponent& c : m_connected_components ) std::cout << "_ " << c.size << std::endl;
        //

        if( num_connected_components != m_connected_components.size() ) throw std::logic_error("internal error");
    }

    int Detector::find_connected_component(int seed, int component)
    {
        if( m_points[seed].connected_component >= 0 ) throw std::logic_error("internal error");
        if( m_points[seed].num_neighbors != 4 ) throw std::logic_error("internal error");

        const int dx[4] = {1, 0, -1, 0};
        const int dy[4] = {0, 1, 0, -1};

        m_points[seed].connected_component = component;
        m_points[seed].coords2d[0] = 0;
        m_points[seed].coords2d[1] = 0;
        orient_neighborhood(seed, 0, 0);

        std::queue<int> queue;
        queue.push(seed);

        int ret = 1;

        //
        cv::Scalar col( rand()%256, rand()%256, rand()%256 );
        //

        while(queue.empty() == false)
        {
            const int idx = queue.front();
            queue.pop();

            SamplePoint& point = m_points[idx];

            //
            {
                std::string text = std::to_string(point.coords2d[0]) + " ; " + std::to_string(point.coords2d[1]);
                cv::circle(m_debug_image, point.keypoint.pt, 4, col, -1);
                cv::putText(
                    m_debug_image,
                    text,
                    point.keypoint.pt,
                    cv::FONT_HERSHEY_PLAIN,
                    1.0, cv::Scalar(255,0,0), 2);
            }
            //

            if( point.num_neighbors != 4 ) throw std::logic_error("internal error");
            if( point.connected_component != component ) throw std::logic_error("internal error");

            for( int neigh_id=0; neigh_id<4; neigh_id++ )
            {
                const int other_idx = point.neighbors[neigh_id];
                SamplePoint& other_point = m_points[other_idx];

                if( other_point.connected_component != component && other_point.num_neighbors == 4 )
                {
                    if( other_point.connected_component >= 0 ) throw std::logic_error("internal error");

                    queue.push(other_idx);

                    ret++;

                    other_point.connected_component = component;

                    other_point.coords2d[0] = point.coords2d[0] + dx[neigh_id];
                    other_point.coords2d[1] = point.coords2d[1] + dy[neigh_id];

                    bool go_on = true;
                    for(int other_neigh_id=0; go_on && other_neigh_id<4; other_neigh_id++)
                    {
                        if( other_point.neighbors[other_neigh_id] == idx )
                        {
                            orient_neighborhood(other_idx, other_neigh_id, (neigh_id+2)%4);
                            go_on = false;
                        }
                    }

                    if(go_on) throw std::logic_error("internal error");
                }
                /*
                else if( m_points[idx_other].num_neighbors == 1 )
                {
                    if( m_points[idx_other].connected_component >= 0 ) throw std::logic_error("internal error");
                    //
                    cv::circle(m_debug_image, m_points[idx].keypoint.pt, 4, col, -1);
                    //
                    m_points[idx_other].connected_component = component;
                }
                */
            }
        }

        return ret;
    }

    void Detector::orient_neighborhood(int idx, int pre, int post)
    {
        SamplePoint& pt = m_points[idx];

        if( pre != post )
        {
            std::swap( pt.neighbors[pre], pt.neighbors[post] );
            std::swap( pt.neighbor_types[pre], pt.neighbor_types[post] );
        }

        const int plus1 = (post+1)%4;
        const int plus2 = (post+2)%4;
        const int plus3 = (post+3)%4;

        if( pt.neighbor_types[plus1] == pt.neighbor_types[post] )
        {
            std::swap( pt.neighbors[plus2], pt.neighbors[plus1] );
            std::swap( pt.neighbor_types[plus2], pt.neighbor_types[plus1] );
        }
        else if( pt.neighbor_types[plus3] == pt.neighbor_types[post] )
        {
            std::swap( pt.neighbors[plus2], pt.neighbors[plus3] );
            std::swap( pt.neighbor_types[plus2], pt.neighbor_types[plus3] );
        }

        const cv::Vec2f two_to_zero = m_points[pt.neighbors[post]].keypoint.pt - m_points[pt.neighbors[plus2]].keypoint.pt;
        const cv::Vec2f two_to_one = m_points[pt.neighbors[plus1]].keypoint.pt - m_points[pt.neighbors[plus2]].keypoint.pt;

        const float dot_product = two_to_one(1) * two_to_zero(0) - two_to_one(0) * two_to_zero(1);

        if( dot_product < 0.0 )
        {
            std::swap( pt.neighbors[plus1], pt.neighbors[plus3] );
            std::swap( pt.neighbor_types[plus1], pt.neighbor_types[plus3] );
        }

        if(
            pt.neighbor_types[0] != pt.neighbor_types[2] ||
            pt.neighbor_types[1] != pt.neighbor_types[3] ||
            pt.neighbor_types[0] == pt.neighbor_types[1] ||
            pt.neighbor_types[2] == pt.neighbor_types[3] )
        {
            throw std::runtime_error("internal error !");
        }
    }
}

