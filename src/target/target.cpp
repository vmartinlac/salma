#include <chrono>
#include <cmath>
#include <queue>
#include <algorithm>
#include <iostream>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <nanoflann.hpp>
#include "target.h"
#include "utils.h"

#define TARGET_DETECTOR_DEBUG

namespace target {

    Detector::Detector() : m_kpl_adapter(&m_points) { }

    bool Detector::run( const cv::Mat& image, KindOfTarget target, float case_side_length, cv::Mat& samples )
    {
        std::chrono::time_point<std::chrono::system_clock> t0 = std::chrono::system_clock::now();

        m_target = target;
        m_case_side_length = case_side_length;

        m_image = &image;
#ifdef TARGET_DETECTOR_DEBUG
        cv::imwrite("10_original.png", *m_image);
#endif

        std::cout << "Converting to greyscale..." << std::endl;
        cv::cvtColor(image, m_greyscale, CV_BGR2GRAY);
#ifdef TARGET_DETECTOR_DEBUG
        cv::imwrite("20_greyscale.png", m_greyscale);
#endif

        std::cout << "Thresholding..." << std::endl;
        cv::threshold(m_greyscale, m_thresh, 100, 255, cv::THRESH_BINARY_INV);
#ifdef TARGET_DETECTOR_DEBUG
        cv::imwrite("30_threshold.png", m_thresh);
#endif

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

        bool ret = false;

        if( target == TWO_PLANES )
        {
            std::cout << "Finding folding line..." << std::endl;
            find_folding_line();

            std::cout << "Storing results..." << std::endl;
            store_results_two_planes(samples);
        }
        else
        {
            std::cout << "Storing results..." << std::endl;
            store_results_one_plane(samples);
        }

#ifdef TARGET_DETECTOR_DEBUG
        {
            cv::Mat debug = m_image->clone();
            for(SamplePoint& pt : m_points)
            {
                if( pt.has_coords_3d )
                {
                    cv::Scalar col;

                    if( m_target == ONE_PLANE )
                    {
                        col = cv::Scalar(255,0,0);
                    }
                    else
                    {
                        const int y =
                            m_folding_line.rotation[1][0]*pt.coords2d[0] +
                            m_folding_line.rotation[1][1]*pt.coords2d[1] +
                            m_folding_line.translation[1];

                        if( y > 0 )
                        {
                            col = cv::Scalar(255,0,0);
                        }
                        else if( y < 0 )
                        {
                            col = cv::Scalar(0,255,0);
                        }
                        else
                        {
                            col = cv::Scalar(0,0,255);
                        }
                    }

                    //cv::circle(debug, pt.keypoint.pt, radius, cv::Scalar(255, 0, 0), -1);

                    std::string text =
                        std::to_string((int)pt.coords_3d.x) + " ; " +
                        std::to_string((int)pt.coords_3d.y) + " ; " +
                        std::to_string((int)pt.coords_3d.z);

                    cv::putText( debug, text, pt.keypoint.pt, cv::FONT_HERSHEY_PLAIN, 1.0, col, 2);
                }
            }
            cv::imwrite("80_result.png", debug);
        }
#endif

        std::chrono::time_point<std::chrono::system_clock> t1 = std::chrono::system_clock::now();

        const int dt = std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count();

        std::cout << "Computation time: " << dt << " ms" << std::endl;
        std::cout << "Framerate: " << 1.0e3f/float(dt) << " Hz" << std::endl;

        return ret;
    }

    void Detector::build_kdtree()
    {
        m_kdtree.reset(new KDTree(2, m_kpl_adapter));
        m_kdtree->buildIndex();
    }

    void Detector::detect_corners()
    {
        cv::Ptr<cv::GFTTDetector> detector = cv::GFTTDetector::create();

        detector->setMinDistance(double(m_image->cols)*20.0/1024.0); // 30.0
        detector->setMaxFeatures(800);

        std::vector<cv::KeyPoint> keypoints;
        detector->detect(*m_image, keypoints);

#ifdef TARGET_DETECTOR_DEBUG
        {
            cv::Mat debug;
            cv::drawKeypoints(*m_image, keypoints, debug);
            cv::imwrite("40_keypoints.png", debug);
        }
#endif

        m_points.clear();

        for( const cv::KeyPoint& kp : keypoints )
        {
            if( filter_keypoint(kp) )
            {
                m_points.emplace_back(kp);
            }
        }
#ifdef TARGET_DETECTOR_DEBUG
        {
            const int radius = 4*m_image->cols/640;
            cv::Mat debug = m_image->clone();
            for( SamplePoint& pt : m_points )
            {
                cv::circle(debug, pt.keypoint.pt, radius, cv::Scalar(255, 0, 0), -1);
            }
            cv::imwrite("50_keypoints_post_filtering.png", debug);
        }
#endif
    }

    bool Detector::filter_keypoint(const cv::KeyPoint& kp)
    {
        if( m_thresh.type() != CV_8U) throw std::runtime_error("bad type of image");

        //const float radius = 15.0;
        const float radius = float(m_image->cols)*20.0/2560.0;

        const float dalpha = 1.5 / radius;

        int symm = 0;
        int antisymm = 0;
        int white = 0;
        int black = 0;

        bool inside = true;
        const cv::Rect viewport( cv::Point2i(0,0), m_thresh.size() );

        for(float alpha = 0.0; inside && alpha<M_PI; alpha += dalpha)
        {
            const float dx = radius*cos(alpha);
            const float dy = radius*sin(alpha);

            const cv::Point P1( kp.pt.x + dx, kp.pt.y + dy );
            const cv::Point P2( kp.pt.x - dx, kp.pt.y - dy );

            if( viewport.contains(P1) && viewport.contains(P2) )
            {
                const uint8_t val1 = m_thresh.at<uint8_t>(P1);
                const uint8_t val2 = m_thresh.at<uint8_t>(P2);

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
            else
            {
                inside = false;
            }
        }

        if( inside )
        {
            const float ratio1 = float(symm) / float(antisymm + symm);
            const float ratio2 = float(white) / float(white + black);

            const float alpha = 0.35;
            const float beta = 0.7;
            return ( ratio1 > beta && 0.5-alpha < ratio2 && ratio2 < 0.5+alpha );
        }
        else
        {
            return false;
        }
    }

    void Detector::detect_neighbors()
    {
        for(int idx=0; idx<m_points.size(); idx++)
        {
            std::vector<int> neighbors;

            find_k_nearest_neighbors(idx, 20, neighbors);

            bool erase = false;
            int num_BW = 0;
            int num_WB = 0;

            for(int other_idx : neighbors)
            {
                KindOfLine kind;
                if( filter_line( m_points[idx].keypoint.pt, m_points[other_idx].keypoint.pt, kind ) && kind != LINE_NONE )
                {
                    int destination_idx = -1;

                    if( kind == LINE_BW && num_BW == 0 )
                    {
                        destination_idx = 0;
                        num_BW++;
                    }
                    else if( kind == LINE_BW && num_BW == 1 )
                    {
                        destination_idx = 2;
                        num_BW++;
                    }
                    else if( kind == LINE_WB && num_WB == 0 )
                    {
                        destination_idx = 1;
                        num_WB++;
                    }
                    else if( kind == LINE_WB && num_WB == 1 )
                    {
                        destination_idx = 3;
                        num_WB++;
                    }
                    else
                    {
                        erase = true;
                        destination_idx = -1;
                    }

                    if( destination_idx >= 0 )
                    {
                        m_points[idx].neighbors[destination_idx] = other_idx;
                        m_points[idx].neighbor_types[destination_idx] = kind;
                    }
                }
            }

            if(erase)
            {
                for(int i=0; i<4; i++)
                {
                    m_points[idx].neighbors[i] = -1;
                    m_points[idx].neighbor_types[i] = LINE_NONE;
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
        //const double l = 10.0;
        const double l = double(m_image->cols)*10.0/2560.0;
        const double alpha = 0.20;
        const double beta = 0.20;

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

        // if there is only one plane, make sure that opposite neighbors of A are symmetric wrt A

        if( m_target == ONE_PLANE )
        {
            for(SamplePoint& pt : m_points)
            {
                for(int i=0; i<2; i++)
                {
                    const int id_a = i;
                    const int id_b = (i+2)%4;

                    bool remove = false;

                    if( pt.neighbor_types[id_a] != LINE_NONE && pt.neighbor_types[id_b] != LINE_NONE )
                    {
                        const cv::Point2f delta_a = m_points[ pt.neighbors[id_a] ].keypoint.pt - pt.keypoint.pt;
                        const cv::Point2f delta_b = m_points[ pt.neighbors[id_b] ].keypoint.pt - pt.keypoint.pt;

                        const double dot_product = delta_a.dot(delta_b) / ( cv::norm(delta_a)*cv::norm(delta_b) );
                        const double threshold = cos(M_PI*174.0/180.0);

                        if( dot_product > threshold )
                        {
                            remove = true;
                        }
                    }

                    if( remove )
                    {
                        pt.neighbor_types[id_a] = LINE_NONE;
                        pt.neighbor_types[id_b] = LINE_NONE;
                        pt.neighbors[id_a] = -1;
                        pt.neighbors[id_b] = -1;
                    }
                }
            }
        }

        // make the graph symmetric.

        for( int idx = 0; idx<m_points.size(); idx++ )
        {
            SamplePoint& pt = m_points[idx];

            for(int i=0; i<4; i++)
            {
                const int idx2 = pt.neighbors[i];

                if( idx2 >= 0 && pt.neighbor_types[i] == LINE_NONE ) throw std::logic_error("internal error");
                if( idx2 < 0 && pt.neighbor_types[i] != LINE_NONE ) throw std::logic_error("internal error");

                if( idx2 >= 0 )
                {
                    const bool asymmetric =
                        m_points[idx2].neighbors[0] != idx &&
                        m_points[idx2].neighbors[1] != idx &&
                        m_points[idx2].neighbors[2] != idx &&
                        m_points[idx2].neighbors[3] != idx;

                    if( asymmetric )
                    {
                        pt.neighbors[i] = -1;
                        pt.neighbor_types[i] = LINE_NONE;
                    }
                }
            }
        }

#ifdef TARGET_DETECTOR_DEBUG
        {
            const int radius = 4*m_image->cols/640;
            cv::Mat debug = m_image->clone();
            for(SamplePoint& pt : m_points)
            {
                cv::circle(debug, pt.keypoint.pt, radius, cv::Scalar(0,255,0), -1);
            }
            for(SamplePoint& pt : m_points)
            {
                for(int neigh_id=0; neigh_id<4; neigh_id++)
                {
                    const int idx2 = pt.neighbors[neigh_id];

                    if( idx2 >= 0 )
                    {
                        SamplePoint& other_pt = m_points[idx2];

                        cv::line(debug, pt.keypoint.pt, 0.5*(pt.keypoint.pt+other_pt.keypoint.pt), cv::Scalar(255,0,0), 2);
                    }
                }
            }
            cv::imwrite("60_post_symmetrization.png", debug);
        }
#endif
    }

    void Detector::compute_connected_components()
    {
        // unmark all points.

        for( SamplePoint& pt : m_points )
        {
            pt.connected_component = -1;
        }

        // compute connected components.

        m_biggest_connected_component = -1;
        int num_connected_components = 0;
        m_connected_components.clear();

        for(int idx = 0; idx<m_points.size(); idx++)
        {
            if( m_points[idx].full_neighborhood() && m_points[idx].connected_component < 0 )
            {
                const int size = find_connected_component(idx, num_connected_components);

                m_connected_components.emplace_back( idx, size );

                if( num_connected_components == 0 || m_connected_components[m_biggest_connected_component].size < size )
                {
                    m_biggest_connected_component = num_connected_components;
                }

                num_connected_components++;
            }
        }

        if( num_connected_components != m_connected_components.size() ) throw std::logic_error("internal error");

#ifdef TARGET_DETECTOR_DEBUG
        {
            cv::Mat debug = m_image->clone();

            std::vector<cv::Scalar> colors(m_connected_components.size());

            std::generate( colors.begin(), colors.end(), [] () { return cv::Scalar( rand()%255, rand()%255, rand()%255 ); } );

            const int radius = 4*m_image->cols/640;

            for( SamplePoint& pt : m_points )
            {
                if( pt.connected_component >= 0 )
                {
                    cv::circle(debug, pt.keypoint.pt, radius, colors[pt.connected_component], -1);
                    std::string text = std::to_string(pt.coords2d[0]) + " ; " + std::to_string(pt.coords2d[1]);
                    cv::putText( debug, text, pt.keypoint.pt, cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255,0,0), 2);
                }
            }

            cv::imwrite("70_post_connected_components.png", debug);
        }
#endif
    }

    int Detector::find_connected_component(int seed, int component)
    {
        if( m_points[seed].connected_component >= 0 ) throw std::logic_error("internal error");
        if( m_points[seed].full_neighborhood() == false ) throw std::logic_error("internal error");

        const int dx[4] = {1, 0, -1, 0};
        const int dy[4] = {0, 1, 0, -1};

        orient_myself(seed);
        m_points[seed].connected_component = component;
        m_points[seed].coords2d[0] = 0;
        m_points[seed].coords2d[1] = 0;

        std::queue<int> queue;
        queue.push(seed);

        int ret = 1;

        while(queue.empty() == false)
        {
            const int idx = queue.front();
            queue.pop();

            SamplePoint& point = m_points[idx];

            if( point.connected_component != component ) throw std::logic_error("internal error");

            for( int neigh_id=0; neigh_id<4; neigh_id++ )
            {
                const int other_idx = point.neighbors[neigh_id];

                if( other_idx >= 0 )
                {
                    SamplePoint& other_point = m_points[other_idx];

                    if( other_point.connected_component != component )
                    {
                        if( other_point.connected_component >= 0 ) throw std::logic_error("internal error");

                        orient_my_neighbor(idx, neigh_id);
                        other_point.coords2d[0] = point.coords2d[0] + dx[neigh_id];
                        other_point.coords2d[1] = point.coords2d[1] + dy[neigh_id];
                        other_point.connected_component = component;

                        queue.push(other_idx);
                        ret++;
                    }
                }
            }
        }

        return ret;
    }

    void Detector::orient_myself(int idx)
    {
        const int pre = 0;
        const int post = 0;

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

    void Detector::orient_my_neighbor(int idx, int neigh_id)
    {
        SamplePoint& pt = m_points[ m_points[idx].neighbors[neigh_id] ];

        const int plus1 = (neigh_id+1)%4;
        const int plus2 = (neigh_id+2)%4;
        const int plus3 = (neigh_id+3)%4;

        // set the right neighbor at plus2.

        bool go_on = true;
        for(int i=0; go_on && i<4; i++)
        {
            if( pt.neighbors[i] == idx )
            {
                if( i != plus2 )
                {
                    std::swap( pt.neighbors[i], pt.neighbors[plus2] );
                    std::swap( pt.neighbor_types[i], pt.neighbor_types[plus2] );
                }
                go_on = false;
            }
        }

        if( go_on ) throw std::logic_error("internal error");
        if( pt.neighbors[plus2] != idx ) throw std::logic_error("internal error");
        if( pt.neighbor_types[plus2] == m_points[idx].neighbor_types[neigh_id] ) throw std::logic_error("internal error");
        if( pt.neighbor_types[plus2] == LINE_NONE ) throw std::logic_error("internal error");

        // set the right neighbor at neigh_id.

        {
            int choice = -1;

            for(int i=1; i<4; i++)
            {
                const int candidate = (plus2+i)%4;

                if(choice < 0)
                {
                    if( pt.neighbor_types[candidate] == LINE_NONE || pt.neighbor_types[candidate] == pt.neighbor_types[plus2] )
                    {
                        choice = candidate;
                    }
                }
                else if( pt.neighbor_types[choice] == LINE_NONE )
                {
                    if( pt.neighbor_types[candidate] == pt.neighbor_types[plus2] )
                    {
                        choice = candidate;
                    }
                }
                else if( pt.neighbor_types[choice] == pt.neighbor_types[plus2] )
                {
                    if( pt.neighbor_types[candidate] == pt.neighbor_types[plus2] )
                    {
                        throw std::logic_error("internal error");
                    }
                }
            }

            if(choice < 0) throw std::logic_error("internal error");

            if( neigh_id != choice )
            {
                std::swap( pt.neighbors[neigh_id], pt.neighbors[choice] );
                std::swap( pt.neighbor_types[neigh_id], pt.neighbor_types[choice] );
            }
        }

        // the the right plus1 and plus3.

        {
            cv::Point2f A = m_points[idx].keypoint.pt;
            cv::Point2f B = pt.keypoint.pt;
            cv::Vec2f AB = B - A;
            cv::Vec2f N( -AB(1), AB(0) );

            bool permute = false;

            if( pt.neighbor_types[plus1] != LINE_NONE )
            {
                cv::Point2f C = m_points[ pt.neighbors[plus1] ].keypoint.pt;

                const float dot_product = N.dot(C-B);
                permute = (dot_product < 0.0);
            }
            else if( pt.neighbor_types[plus3] != LINE_NONE )
            {
                cv::Point2f C = m_points[ pt.neighbors[plus3] ].keypoint.pt;

                const float dot_product = N.dot(C-B);
                permute = (dot_product > 0.0);
            }

            if( permute )
            {
                std::swap( pt.neighbor_types[plus1], pt.neighbor_types[plus3] );
                std::swap( pt.neighbors[plus1], pt.neighbors[plus3] );
            }
        }

        // check that everything looks OK.

        if( m_points[idx].neighbor_types[neigh_id] == LINE_NONE ) throw std::logic_error("internal error");
        if( pt.neighbor_types[plus2] == LINE_NONE || pt.neighbor_types[plus2] == m_points[idx].neighbor_types[neigh_id] ) throw std::logic_error("internal error");
        if( pt.neighbor_types[neigh_id] != LINE_NONE && pt.neighbor_types[neigh_id] != pt.neighbor_types[plus2] ) throw std::logic_error("internal error");
        if( pt.neighbor_types[plus1] == pt.neighbor_types[plus2] ) throw std::logic_error("internal error");
        if( pt.neighbor_types[plus3] == pt.neighbor_types[plus2] ) throw std::logic_error("internal error");
    }

    void Detector::find_folding_line()
    {
        std::vector<int> points; // points which are folded.

        bool folded[2] = { false, false };

        for(int idx=0; idx<m_points.size(); idx++)
        {
            SamplePoint& pt = m_points[idx];

            if( pt.connected_component == m_biggest_connected_component )
            {
                bool add = false;

                for(int i=0; i<2; i++)
                {
                    const int id_a = i;
                    const int id_b = (i+2)%4;

                    if( pt.neighbor_types[id_a] != LINE_NONE && pt.neighbor_types[id_b] != LINE_NONE )
                    {
                        const cv::Point2f delta_a = m_points[ pt.neighbors[id_a] ].keypoint.pt - pt.keypoint.pt;
                        const cv::Point2f delta_b = m_points[ pt.neighbors[id_b] ].keypoint.pt - pt.keypoint.pt;

                        const double dot_product = delta_a.dot(delta_b) / ( cv::norm(delta_a)*cv::norm(delta_b) );
                        const double threshold = cos(M_PI*170.0/180.0);

                        if( dot_product > threshold )
                        {
                            add = true;
                            folded[i] = true;
                        }
                    }
                }

                if( add )
                {
                    points.push_back(idx);
                }
            }
        }

        bool has_constant_x = false;
        bool has_constant_y = false;
        int constant_x = 0;
        int constant_y = 0;

        if( points.size() == 1 )
        {
            if( folded[0] == folded[1] )
            {
                m_folding_line.found = false;
            }
            else
            {
                has_constant_x = folded[0];
                has_constant_y = folded[1];
                constant_x = m_points[points.front()].coords2d[0];
                constant_y = m_points[points.front()].coords2d[1];
                m_folding_line.found = true;
            }
        }
        else if( points.size() >= 2 )
        {
            has_constant_x = true;
            has_constant_y = true;
            constant_x = m_points[points.front()].coords2d[0];
            constant_y = m_points[points.front()].coords2d[1];

            for( int idx : points )
            {
                if( has_constant_x && constant_x != m_points[idx].coords2d[0] )
                {
                    has_constant_x = false;
                }
                if( has_constant_y && constant_y != m_points[idx].coords2d[1] )
                {
                    has_constant_y = false;
                }
            }

            if( has_constant_x && has_constant_y )
            {
                throw std::logic_error("internal error");
            }

            m_folding_line.found = (has_constant_x != has_constant_y);
        }
        else
        {
            m_folding_line.found = false;
        }

        if( m_folding_line.found )
        {
            if( has_constant_x == has_constant_y) throw std::logic_error("internal error");

            if(has_constant_x)
            {
                std::cout << "Folding line is x = " << constant_x << std::endl;
                m_folding_line.rotation[0][0] = 0;
                m_folding_line.rotation[0][1] = 1;
                m_folding_line.rotation[1][0] = 1;
                m_folding_line.rotation[1][1] = 0;
                m_folding_line.translation[0] = 0;
                m_folding_line.translation[1] = -constant_x;
            }
            else
            {
                std::cout << "Folding line is y = " << constant_y << std::endl;
                m_folding_line.rotation[0][0] = 1;
                m_folding_line.rotation[0][1] = 0;
                m_folding_line.rotation[1][0] = 0;
                m_folding_line.rotation[1][1] = 1;
                m_folding_line.translation[0] = 0;
                m_folding_line.translation[1] = -constant_y;
            }
        }
    }

    bool Detector::store_results_two_planes(cv::Mat& samples)
    {
        bool ret = false;

        if( m_folding_line.found && m_biggest_connected_component >= 0 )
        {
            int count = 0;
            for( SamplePoint& pt : m_points )
            {
                pt.has_coords_3d = (pt.connected_component == m_biggest_connected_component);
                if( pt.has_coords_3d )
                {
                    const int x =
                        m_folding_line.rotation[0][0]*pt.coords2d[0] +
                        m_folding_line.rotation[0][1]*pt.coords2d[1] +
                        m_folding_line.translation[0];

                    const int y =
                        m_folding_line.rotation[1][0]*pt.coords2d[0] +
                        m_folding_line.rotation[1][1]*pt.coords2d[1] +
                        m_folding_line.translation[1];

                    if( y >= 0 )
                    {
                        pt.coords_3d = cv::Point3f(
                            m_case_side_length*float(x),
                            m_case_side_length*float(y),
                            0.0 );
                    }
                    else
                    {
                        pt.coords_3d = cv::Point3f(
                            m_case_side_length*float(x),
                            0.0,
                            m_case_side_length*float(-y) );
                    }

                    count++;
                }
            }

            samples.create( count, 5, CV_32F );

            for(SamplePoint& pt : m_points)
            {
                if( pt.has_coords_3d )
                {
                    count--;

                    if( count < 0 ) throw std::logic_error("internal error");

                    samples.at<float>(count, 0) = pt.keypoint.pt.x;
                    samples.at<float>(count, 1) = pt.keypoint.pt.y;
                    samples.at<float>(count, 2) = pt.coords_3d.x;
                    samples.at<float>(count, 3) = pt.coords_3d.y;
                    samples.at<float>(count, 4) = pt.coords_3d.z;
                }
            }

            ret = true;
        }

        return ret;
    }

    bool Detector::store_results_one_plane(cv::Mat& samples)
    {
        if( m_biggest_connected_component >= 0 )
        {
            int count = m_connected_components[m_biggest_connected_component].size;

            samples.create(count, 5, CV_32F);

            for(SamplePoint& pt : m_points)
            {
                if( pt.connected_component == m_biggest_connected_component )
                {
                    count--;

                    if(count < 0) throw std::logic_error("internal error");

                    pt.has_coords_3d = true;

                    pt.coords_3d = cv::Point3f(
                        m_case_side_length*float(pt.coords2d[0]),
                        m_case_side_length*float(pt.coords2d[1]),
                        0.0 );

                    samples.at<float>(count, 0) = pt.keypoint.pt.x;
                    samples.at<float>(count, 1) = pt.keypoint.pt.x;
                    samples.at<float>(count, 2) = pt.coords_3d.x;
                    samples.at<float>(count, 3) = pt.coords_3d.y;
                    samples.at<float>(count, 4) = pt.coords_3d.z;
                }
            }

            return true;
        }
        else
        {
            return false;
        }
    }
}
