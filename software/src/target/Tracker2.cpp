#include <random>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <map>
#include "Tracker2.h"

static void segment(
    const cv::Mat& image)
{
    std::vector<cv::Point> neighbors;
    
    {
        const int radius = 5;

        for(int di=-radius; di<=radius; di++)
        {
            for(int dj=-radius; dj<=radius; dj++)
            {
                if(di*di + dj*dj <= radius*radius && (di != 0 || dj != 0))
                {
                    neighbors.emplace_back(di,dj);
                }
            }
        }
    }

    const int N = image.rows;
    const int M = image.cols;

    cv::Mat a( N, M, CV_32SC1 );
    a = -1;

    const cv::Rect roi(0, 0, M, N);

    std::cout << "A..." << std::endl;
    for(cv::MatIterator_<int32_t> it=a.begin<int32_t>(); it!=a.end<int32_t>(); it++)
    {
        const cv::Point2i c = it.pos();

        *it = c.y * M + c.x;

        int32_t best = image.at<int32_t>(it.pos());

        for(const cv::Point& delta : neighbors)
        {
            const cv::Point2i o = c + delta;

            if(roi.contains(o))
            {
                const int32_t level = image.at<int32_t>(o);

                if(level > best)
                {
                    *it = o.y * M + o.x;
                    best = level;
                }
            }
        }
    }

    std::vector<int> segments;

    std::cout << "B..." << std::endl;
    for(cv::MatIterator_<int32_t> it=a.begin<int32_t>(); it!=a.end<int32_t>(); it++)
    {
        bool go_on = true;

        const cv::Point pt = it.pos();

        const int32_t me = pt.y * M + pt.x;

        while(go_on)
        {
            if( *it == me )
            {
                go_on = false;
                segments.push_back(me);
            }
            else
            {
                cv::Point parent;
                parent.x = *it % M;
                parent.y = *it / M;

                const int32_t parent_of_parent = a.at<int32_t>(parent);

                if( parent_of_parent == *it )
                {
                    go_on = false;
                }
                else
                {
                    *it = parent_of_parent;
                }
            }
        }
    }

    std::cout << "C..." << std::endl;
    std::default_random_engine engine;
    std::uniform_int_distribution<int> distrib(0, 255);
    auto proc = [&engine, &distrib] ()
    {
        const int r = distrib(engine);
        const int g = distrib(engine);
        const int b = distrib(engine);
        return cv::Vec<uint8_t,3>(r,b,g);
    };

    std::vector<cv::Vec<uint8_t,3> > colors(segments.size());
    std::generate(colors.begin(), colors.end(), proc);

    std::cout << segments.size() << std::endl;

    cv::Mat im(N, M, CV_8UC3);

    for(cv::MatIterator_<int32_t> it=a.begin<int32_t>(); it!=a.end<int32_t>(); it++)
    {
        int i = 0;
        while(i<segments.size() && segments[i] != *it)
        {
            i++;
        }

        if(i<segments.size())
        {
            im.at< cv::Vec<uint8_t,3> >() = colors[i];
        }
        else
        {
        }
    }
}

target::Tracker2::Tracker2()
{
}

bool target::Tracker2::track( const cv::Mat& image, bool absolute_pose )
{
    cv::imwrite("debug_output/10_original.png", image);

    std::cout << "Converting to grayscale..." << std::endl;
    cv::Mat gray;
    cv::cvtColor(image, gray, CV_BGR2GRAY);
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
        [] (uint8_t x) { return (x == 0) ? 1 : 0; } );
    cv::imwrite("debug_output/40_threshold.png", thresh_b);

    std::cout << "Distance transform..." << std::endl;
    cv::Mat dist_a;
    cv::Mat dist_b;
    cv::distanceTransform(thresh_a, dist_a, cv::DIST_L2, 5, CV_32S);
    cv::distanceTransform(thresh_b, dist_b, cv::DIST_L2, 5, CV_32S);

    std::cout << "Segment..." << std::endl;
    segment(dist_a);
    segment(dist_b);

    return false;
}

void target::Tracker2::clear()
{
    TrackerBase::clear();
}

