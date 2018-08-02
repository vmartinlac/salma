#include <QApplication>
#include <array>
#include <iostream>
#include "Camera.h"
#include "Image.h"
//#include "SLAMEngine.h"

#include <algorithm>

#include <QThread>
#include <Eigen/Eigen>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "utils.h"

//#include <boost/pending/disjoint_sets.hpp>

struct SLAMEngineParameters
{
    Eigen::Matrix3d calibration_matrix;
    int patch_size;
    int num_depth_hypotheses;
};

/*
bool SLAMEngineParameters::load_from_file(const std::string& path)
{
    cv::FileStorage file("", cv::FileStorage::READ);

    if( file.isOpened() == false ) throw std::runtime_error("Failed to open the file");

    cv::Mat K;
    file["calibration_matrix"] >> K;

    file.release();
}
*/

class SLAMEngine : public QThread
{
public:

    SLAMEngine(const SLAMEngineParameters& parameters, Camera* camera) : m_parameters(parameters), m_camera(m_camera)
    {
        ;
    }

    virtual void run()
    {
        m_camera->start();

        while( isInterruptionRequested() == false )
        {
        }

        m_camera->stop();
    }

protected:

    typedef Eigen::Matrix<double, 12, 1> CameraState;

    struct Landmark
    {
        Eigen::Vector3d position;
        cv::Mat patch;
    };

    struct CandidateLandmark
    {
        cv::Mat patch;
        Eigen::Vector3d origin;
        Eigen::Vector3d direction;
        std::vector<float> depth_hypotheses;
    };

protected:

    SLAMEngineParameters m_parameters;
    Camera* m_camera;
    CameraState m_camera_state;
    std::vector<Landmark> m_landmarks;
    Eigen::MatrixXd m_state_covariance;
    std::vector<CandidateLandmark> m_candidate_landmarks;
};

void segment(const cv::Mat& distance, cv::Mat& segmentation, int& num_classes, std::vector<int>& class_sizes)
{
    std::array<cv::Vec2s, 8> neighbors;

    neighbors[0] = cv::Vec2s(-1, 0);
    neighbors[1] = cv::Vec2s(1, 0);
    neighbors[2] = cv::Vec2s(0, -1);
    neighbors[3] = cv::Vec2s(0, 1);
    neighbors[4] = cv::Vec2s(-1, -1);
    neighbors[5] = cv::Vec2s(-1, 1);
    neighbors[6] = cv::Vec2s(1, -1);
    neighbors[7] = cv::Vec2s(1, 1);

    cv::Rect viewport(cv::Point(), distance.size());

    segmentation.create(distance.size(), CV_32S);

    for( cv::MatIterator_<int32_t> it = segmentation.begin<int32_t>(); it != segmentation.end<int32_t>(); it++ )
    {
        cv::Point best = it.pos();
        
        for(cv::Vec2s delta : neighbors)
        {
            cv::Point neighbor( it.pos().x + delta(0), it.pos().y + delta(1) );

            if( neighbor.inside(viewport) && distance.at<float>(neighbor) > distance.at<float>(best) )
            {
                best = neighbor;
            }
        }

        if( best == it.pos() )
        {
            *it = -1;
        }
        else
        {
            *it = best.y * distance.cols + best.x;
        }
    }

    num_classes = 0;
    std::map<int32_t, int> classes;

    for( cv::MatIterator_<int32_t> it = segmentation.begin<int32_t>(); it != segmentation.end<int32_t>(); it++ )
    {
        if(*it == -1)
        {
            const int id = it.pos().y * distance.cols + it.pos().x;
            classes[id] = num_classes;
            num_classes++;
        }
        else
        {
            bool go_on = true;

            while(go_on)
            {
                int x = *it % distance.cols;
                int y = *it / distance.cols;
                int father = segmentation.at<int32_t>(y, x);
                if( father == -1 )
                {
                    go_on = false;
                }
                else
                {
                    *it = father;
                }
            }
        }
    }

    class_sizes.assign(num_classes, 0);

    for( cv::MatIterator_<int32_t> it = segmentation.begin<int32_t>(); it != segmentation.end<int32_t>(); it++ )
    {
        if(*it == -1)
        {
            *it = classes[it.pos().y * distance.cols + it.pos().x];
        }
        else
        {
            *it = classes[*it];
        }

        class_sizes[*it]++;
    }

    /*
    for( cv::MatIterator_<int32_t> it = segmentation.begin<int32_t>(); it != segmentation.end<int32_t>(); it++ )
    {
        *it = (*it)*255/(num_classes-1);
    }
    */
}

template<typename T, int D>
class PriorityFilter
{
public:

    PriorityFilter()
    {
        m_size = 0;
        m_weakest = 0;
    }

    size_t size() { return m_size; }

    T* begin() { return &m_data.front(); }

    T* end() { return &m_data.front() + m_size; }

    void insert(const T& item, float weight)
    {
        if( m_size == 0 )
        {
            m_size = 1;
            m_data.front() = item;
            m_weight.front() = weight;
            m_weakest = 0;
        }
        else if( m_size < D )
        {
            m_data[m_size] = item;
            m_weight[m_size] = weight;
            if( weight > m_weight[m_weakest] )
            {
                m_weakest = m_size;
            }
            m_size++;
        }
        else if( m_size == D )
        {
            if( weight < m_weight[m_weakest] )
            {
                m_data[m_weakest] = item;
                m_weight[m_weakest] = weight;

                m_weakest = 0;

                for(int i=1; i<D; i++)
                {
                    if(m_weight[m_weakest] < m_weight[i])
                    {
                        m_weakest = i;
                    }
                }
            }
        }
        else
        {
            throw std::logic_error("error");
        }
    }

protected:

    std::array<T, D> m_data;
    std::array<float, D> m_weight;
    size_t m_size;
    size_t m_weakest;
};

void compute_corners(
    const cv::Mat& distance,
    const cv::Mat& segmentation,
    int num_classes,
    const std::vector<int>& class_sizes,
    std::vector<cv::Point>& corners)
{
    const int max_corners_per_class = 4;

    std::vector< PriorityFilter<cv::Point,max_corners_per_class> > class_corners(num_classes);

    for( cv::MatConstIterator_<int32_t> it = segmentation.begin<int32_t>(); it != segmentation.end<int32_t>(); it++ )
    {
        class_corners[*it].insert( it.pos(), distance.at<int32_t>(it.pos()) );
    }

    corners.clear();

    for( PriorityFilter<cv::Point,max_corners_per_class>& pf : class_corners )
    {
        std::copy(pf.begin(), pf.end(), std::back_inserter(corners));
    }
}

int main(int num_args, char** args)
{
    QApplication app(num_args, args);

    std::cout << "Loading image..." << std::endl;
    cv::Mat image;
    image = cv::imread("/home/victor/developpement/slam/data/photographies_mire/a.jpg");

    std::cout << "Resizing image..." << std::endl;
    cv::Mat mini;
    cv::resize(image, mini, cv::Size(1024, 768));

    std::cout << "Applying canny filter..." << std::endl;
    cv::Mat canny;
    cv::Canny(mini, canny, 10, 100);
    canny = 255 - canny;

    std::cout << "Computing distance transform..." << std::endl;
    cv::Mat distance;
    cv::Mat labels;
    cv::distanceTransform(canny, distance, labels, cv::DIST_L2, cv::DIST_MASK_PRECISE);

    std::cout << "Computing segmentation..." << std::endl;
    cv::Mat segmentation;
    std::vector<int> class_sizes;
    int nc;
    segment(distance, segmentation, nc, class_sizes);

    cv::Mat tmp = segmentation*255/(nc-1);
    imshow(tmp);

    std::cout << "Computing corners..." << std::endl;
    std::vector<cv::Point> points;
    compute_corners(distance, segmentation, nc, class_sizes, points);
    for(cv::Point pt : points) cv::circle(mini, pt, 3, cv::Scalar(0, 255, 0), 1);

    imshow(mini);

    return 0;
}

