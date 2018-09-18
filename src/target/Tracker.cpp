#include <chrono>
#include <Eigen/Eigen>
#include <cmath>
#include <queue>
#include <algorithm>
#include <iostream>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <nanoflann.hpp>
#include "Tracker.h"

#define TARGET_DETECTOR_DEBUG

namespace target {

    Tracker::Tracker() : m_kpl_adapter(&m_points)
    {
        m_unit_length = 1.0;
        m_found = false;
    }

    bool Tracker::track( const cv::Mat& image )
    {
        std::chrono::time_point<std::chrono::system_clock> t0 = std::chrono::system_clock::now();

        m_image = &image;
#ifdef TARGET_DETECTOR_DEBUG
        cv::imwrite("debug_output/10_original.png", *m_image);
#endif

        // Converting to greyscale.
        cv::cvtColor(image, m_greyscale, CV_BGR2GRAY);
#ifdef TARGET_DETECTOR_DEBUG
        cv::imwrite("debug_output/20_greyscale.png", m_greyscale);
#endif

        // Thresholding
        cv::threshold(m_greyscale, m_thresh, 100, 255, cv::THRESH_BINARY_INV);
#ifdef TARGET_DETECTOR_DEBUG
        cv::imwrite("debug_output/30_threshold.png", m_thresh);
#endif

        // Detecting corners...
        detect_corners();

        // Constructing kdtree...
        build_kdtree();

        // Detecting neighbors...
        detect_neighbors();

        // Enforcing symmetry...
        symmetrize();

        // Computing connected components...
        compute_connected_components();

        // Compute absolute orientation.
        compute_absolute_orientation();

        m_found = save_results();

        std::chrono::time_point<std::chrono::system_clock> t1 = std::chrono::system_clock::now();
        const int dt = std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count();

        std::cout << "Computation time: " << dt << " ms" << std::endl;
        //std::cout << "Framerate: " << 1.0e3f/float(dt) << " Hz" << std::endl;

        return m_found;
    }

    void Tracker::build_kdtree()
    {
        m_kdtree.reset(new KDTree(2, m_kpl_adapter));
        m_kdtree->buildIndex();
    }

    void Tracker::detect_corners()
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
            cv::imwrite("debug_output/40_keypoints.png", debug);
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
            cv::imwrite("debug_output/50_keypoints_post_filtering.png", debug);
        }
#endif
    }

    bool Tracker::filter_keypoint(const cv::KeyPoint& kp)
    {
        if( m_thresh.type() != CV_8U) throw std::runtime_error("bad type of image");

        const float radius = float(m_image->cols)*20.0/2560.0;

        const float margin = 5.0;

        const bool inside =
            0.0 < kp.pt.x - radius - margin &&
            kp.pt.x + radius + margin < float(m_thresh.cols-1) &&
            0.0 < kp.pt.y - radius - margin &&
            kp.pt.y + radius + margin < float(m_thresh.rows-1);

        if( inside )
        {
            const float dalpha = 1.5 / radius;

            int symm = 0;
            int antisymm = 0;
            int white = 0;
            int black = 0;

            for(float alpha = 0.0; alpha<M_PI; alpha += dalpha)
            {
                const float dx = radius*cosf(alpha);
                const float dy = radius*sinf(alpha);

                const cv::Point P1( kp.pt.x + dx, kp.pt.y + dy );
                const cv::Point P2( kp.pt.x - dx, kp.pt.y - dy );

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

    void Tracker::detect_neighbors()
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

    void Tracker::find_k_nearest_neighbors(int index, size_t k, std::vector<int>& neighbors)
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

    bool Tracker::filter_line(const cv::Point2f& A, const cv::Point2f& B, KindOfLine& kind)
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
        const float white_ratio = float(white_left + white_right) / float(total);
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

    void Tracker::symmetrize()
    {

        // if there is only one plane, make sure that opposite neighbors of A are symmetric wrt A

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
            cv::imwrite("debug_output/60_post_symmetrization.png", debug);
        }
#endif
    }

    void Tracker::compute_connected_components()
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

            cv::imwrite("debug_output/70_post_connected_components.png", debug);
        }
#endif
    }

    int Tracker::find_connected_component(int seed, int component)
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

    void Tracker::orient_myself(int idx)
    {
        const int pre = 0;
        const int post = 0;

        //

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
        const cv::Vec2f normal( two_to_zero(1), -two_to_zero(0) );

        const float dot_product = two_to_one(0) * normal(0) + two_to_one(1) * normal(1);

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

    void Tracker::orient_my_neighbor(int idx, int neigh_id)
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
            cv::Vec2f N( AB(1), -AB(0) );

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

    void Tracker::compute_absolute_orientation()
    {
        std::vector<int> circles;
        circles.reserve(3);

#ifdef TARGET_DETECTOR_DEBUG
        cv::Mat debug = m_image->clone();
#endif

        for(int i=0; i<m_points.size(); i++)
        {
            std::array<int,4> cell;

            if( find_cell(i, cell) )
            {
                Eigen::Matrix<double, 8, 9> M;

                cv::Point2f roi_min(
                    int(m_points[i].keypoint.pt.x),
                    int(m_points[i].keypoint.pt.y));

                cv::Point2f roi_max = roi_min;

                for(int k=0; k<4; k++)
                {
                    roi_min.x = std::min(roi_min.x, m_points[cell[k]].keypoint.pt.x);
                    roi_min.y = std::min(roi_min.y, m_points[cell[k]].keypoint.pt.y);
                    roi_max.x = std::max(roi_max.x, m_points[cell[k]].keypoint.pt.x);
                    roi_max.y = std::max(roi_max.y, m_points[cell[k]].keypoint.pt.y);

                    const double pre_u = m_points[cell[k]].keypoint.pt.x;
                    const double pre_v = m_points[cell[k]].keypoint.pt.y;

                    const double post_u = double( m_points[cell[k]].coords2d[0] - m_points[cell[0]].coords2d[0] );
                    const double post_v = double( m_points[cell[k]].coords2d[1] - m_points[cell[0]].coords2d[1] );

                    M(2*k+0, 0) = 0.0;
                    M(2*k+0, 1) = 0.0;
                    M(2*k+0, 2) = 0.0;
                    M(2*k+0, 3) = -pre_u;
                    M(2*k+0, 4) = -pre_v;
                    M(2*k+0, 5) = -1;
                    M(2*k+0, 6) = pre_u*post_v;
                    M(2*k+0, 7) = pre_v*post_v;
                    M(2*k+0, 8) = post_v;

                    M(2*k+1, 0) = pre_u;
                    M(2*k+1, 1) = pre_v;
                    M(2*k+1, 2) = 1.0;
                    M(2*k+1, 3) = 0.0;
                    M(2*k+1, 4) = 0.0;
                    M(2*k+1, 5) = 0.0;
                    M(2*k+1, 6) = -pre_u*post_u;
                    M(2*k+1, 7) = -pre_v*post_u;
                    M(2*k+1, 8) = -post_u;
                }

                Eigen::JacobiSVD<decltype(M)> solver;
                solver.compute(M, Eigen::ComputeFullV);

                Eigen::Matrix<double, 9, 1> coeffs = solver.matrixV().rightCols<1>();

                Eigen::Matrix3d H;
                H(0,0) = coeffs(0);
                H(0,1) = coeffs(1);
                H(0,2) = coeffs(2);
                H(1,0) = coeffs(3);
                H(1,1) = coeffs(4);
                H(1,2) = coeffs(5);
                H(2,0) = coeffs(6);
                H(2,1) = coeffs(7);
                H(2,2) = coeffs(8);

                const bool found_circle = filter_circle(
                    cv::Rect(roi_min, roi_max),
                    H);

                if( found_circle )
                {
                    circles.push_back(i);
                }

#ifdef TARGET_DETECTOR_DEBUG
                {
                    Eigen::Matrix3d inv_H = H.inverse();

                    Eigen::Vector3d A = inv_H * Eigen::Vector3d{0.25, 0.5, 1.0};
                    Eigen::Vector3d B = inv_H * Eigen::Vector3d{0.5, 0.5, 1.0};

                    if( std::fabs(A(2)) > 1.0e-7 && std::fabs(B(2)) > 1.0e-7 )
                    {
                        A /= A(2);
                        B /= B(2);

                        const double radius = (A-B).norm();
                        const cv::Point2f cen( B(0), B(1) );

                        cv::Scalar color;
                        if(found_circle)
                        {
                            color = cv::Scalar(0, 255, 0);
                        }
                        else
                        {
                            color = cv::Scalar(0, 0, 255);
                        }

                        cv::circle(debug, cen, radius, color, 2);
                    }
                }
#endif
            }
        }

#ifdef TARGET_DETECTOR_DEBUG
        cv::imwrite("debug_output/80_find_circles.png", debug);
#endif
    }

    bool Tracker::find_cell_anticlockwise(int point, std::array<int,4>& cell)
    {
        SamplePoint& pt = m_points[point];

        bool ok = ( pt.connected_component == m_biggest_connected_component );

        cell[0] = point;

        if(ok)
        {
            cell[1] = pt.neighbors[0];
            ok = ( cell[1] >= 0 );
        }

        if(ok)
        {
            cell[2] = m_points[cell[1]].neighbors[1];
            ok = ( cell[2] >= 0 );
        }

        if(ok)
        {
            cell[3] = m_points[cell[2]].neighbors[2];
            ok = ( cell[3] >= 0 );
        }

        return ok;
    }

    bool Tracker::find_cell_clockwise(int point, std::array<int,4>& cell)
    {
        SamplePoint& pt = m_points[point];

        bool ok = ( pt.connected_component == m_biggest_connected_component );

        cell[0] = point;

        if(ok)
        {
            cell[3] = pt.neighbors[1];
            ok = ( cell[3] >= 0 );
        }

        if(ok)
        {
            cell[2] = m_points[cell[3]].neighbors[0];
            ok = ( cell[2] >= 0 );
        }

        if(ok)
        {
            cell[1] = m_points[cell[2]].neighbors[3];
            ok = ( cell[1] >= 0 );
        }

        return ok;
    }

    bool Tracker::find_cell(int point, std::array<int,4>& cell)
    {
        return find_cell_clockwise(point, cell) || find_cell_anticlockwise(point, cell);
    }

    bool Tracker::filter_circle(const cv::Rect& roi, const Eigen::Matrix3d& H)
    {
        int outside_white = 0;
        int outside_black = 0;
        int inside_white = 0;
        int inside_black = 0;

        for(int dx=0; dx<roi.width; dx++)
        {
            for(int dy=0; dy<roi.height; dy++)
            {
                //if( (dx+dy)%2 == 0 ) continue;

                cv::Point2i pt(roi.x + dx, roi.y + dy);

                Eigen::Vector3d pre;
                pre(0) = pt.x;
                pre(1) = pt.y;
                pre(2) = 1.0;

                Eigen::Vector3d post = H * pre;

                if( std::fabs(post(2)) > 1.0e-7 )
                {
                    post /= post(2);

                    const double du = post(0) - 0.5;
                    const double dv = post(1) - 0.5;
                    const double radius_squared = du*du + dv*dv;

                    if( radius_squared < 1.0/36.0)
                    {
                        if( m_thresh.at<uint8_t>(pt) )
                        {
                            inside_white++;
                        }
                        else
                        {
                            inside_black++;
                        }
                    }
                    else if( radius_squared < 1.0/9.0 )
                    {
                        if( m_thresh.at<uint8_t>(pt) )
                        {
                            outside_white++;
                        }
                        else
                        {
                            outside_black++;
                        }
                    }
                }
            }
        }

        if( inside_white + inside_black < 15 || outside_white + outside_black < 15 )
        {
            return false;
        }
        else
        {
            const double r1 = double(inside_white) / double(inside_white + inside_black);
            const double r2 = double(outside_white) / double(outside_white + outside_black);

            const double eps = 0.12;

            return (r1 < eps && r2 > 1.0-eps) || (r2 < eps && r1 > 1.0-eps);
        }
    }

    bool Tracker::save_results()
    {
        bool ret = false;

        if( m_biggest_connected_component >= 0 )
        {
            int count = m_connected_components[m_biggest_connected_component].size;

            m_object_points.resize(count);
            m_image_points.resize(count);
            m_image_keypoints.resize(count);

            for(SamplePoint& pt : m_points)
            {
                if( pt.connected_component == m_biggest_connected_component )
                {
                    count--;

                    if(count < 0) throw std::logic_error("internal error");

                    m_object_points[count] = cv::Point3f(
                        m_unit_length*float(pt.coords2d[0]),
                        m_unit_length*float(pt.coords2d[1]),
                        0.0 );

                    m_image_points[count] = pt.keypoint.pt;

                    m_image_keypoints[count] = pt.keypoint;
                }
            }

            if(count != 0) throw std::logic_error("internal error");

            cv::TermCriteria term_criteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS, 20, 1.0e-3);
            cv::cornerSubPix(m_greyscale, m_image_points, cv::Size(5, 5), cv::Size(-1, -1), term_criteria);

            ret = true;
        }
        else
        {
            m_image_points.clear();
            m_object_points.clear();

            ret = false;
        }

        return ret;
    }

    void Tracker::clear()
    {
        m_image_points.clear();
        m_object_points.clear();
        m_connected_components.clear();
        m_kdtree.reset();
        m_points.clear();
        m_greyscale.release();
        m_thresh.release();
    }
}

