#include <Eigen/Eigen>
#include <opencv2/features2d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
#include <utility>
#include "SLAMModule1StereoMatcher.h"
#include "FinitePriorityQueue.h"

SLAMModule1StereoMatcher::SLAMModule1StereoMatcher(SLAMContextPtr con) :
    SLAMModule(SLAM_MODULE1_STEREOMATCHER, con)
{
}

SLAMModule1StereoMatcher::~SLAMModule1StereoMatcher()
{
}

bool SLAMModule1StereoMatcher::init()
{
    SLAMContextPtr con = context();

    mCheckOctave = con->configuration->stereo_matcher.check_octave;

    mCheckSymmetry = con->configuration->stereo_matcher.check_symmetry;

    mCheckLowe = con->configuration->stereo_matcher.check_lowe;
    mLoweRatio = con->configuration->stereo_matcher.lowe_ratio;

    mCheckEpipolar = con->configuration->stereo_matcher.check_epipolar;
    mEpipolarThreshold = con->configuration->stereo_matcher.epipolar_threshold;

    mStereoRigCalibration = con->calibration;

    mFundamentalMatrices[0] = con->calibration->computeFundamentalMatrix(0, 1);
    mFundamentalMatrices[1] = mFundamentalMatrices[0].transpose();

    return true;
}

int SLAMModule1StereoMatcher::matchKeyPoint(SLAMFramePtr f, int view, int i, bool check_symmetry)
{
    const int other_view = (view + 1) % 2;

    /*
    if(context()->configuration->debug)
    {
        cv::Mat image = f->views[view].image.clone();
        cv::Mat other_image = f->views[other_view].image.clone();

        cv::circle(image, f->views[view].keypoints[i].pt, 5, cv::Scalar(0,255,0), -1);

        Eigen::Vector3d pointi;
        pointi <<
            mUndistortedPoints[view][i].x,
            mUndistortedPoints[view][i].y,
            1.0;

        Eigen::Vector3d line = mFundamentalMatrices[view] * pointi;

        const double l = line.head<2>().norm();

        if( l > 1.0e-7 )
        {
            line /= l;

            Eigen::Vector2d normal = line.head<2>();

            Eigen::Vector2d tangent;
            tangent.x() = -normal.y();
            tangent.y() = normal.x();

            const Eigen::Vector2d origin = - line.z() * normal;
            const double delta = mStereoRigCalibration->cameras[0].calibration->image_size.width + mStereoRigCalibration->cameras[0].calibration->image_size.height;
            const Eigen::Vector2d A = origin + delta * tangent;
            const Eigen::Vector2d B = origin - delta * tangent;

            const cv::Point2f cvA(A.x(), A.y());
            const cv::Point2f cvB(B.x(), B.y());

            cv::line(other_image, cvA, cvB, cv::Scalar(0,255,0), 3);

            cv::Mat output(std::max(image.rows, other_image.rows), image.cols+other_image.cols, CV_8UC3);
            image.copyTo( output( cv::Rect(0, 0, image.cols, image.rows) ) );
            other_image.copyTo( output( cv::Rect(image.cols, 0, other_image.cols, other_image.rows) ) );

            context()->debug->saveImage(f->id, "STEREOMATCHER_epipolar_line.png", output);
        }
    }
    */

    FinitePriorityQueueF<int, double, 2> queue;

    for(int j=0; j<f->views[other_view].keypoints.size(); j++)
    {
        bool ok = true;

        if(ok && mCheckOctave)
        {
            ok = ( f->views[view].keypoints[i].octave == f->views[other_view].keypoints[j].octave );
        }

        if(ok)
        {
            const double distance = cv::norm(
                f->views[view].descriptors.row(i),
                f->views[other_view].descriptors.row(j));
            queue.push( j, -distance);
        }
    }

    int ret = -1;

    if( queue.size() >= 2 && mCheckLowe )
    {
        const int j1 = queue.top();
        const double p1 = queue.top_priority();

        queue.pop();

        const int j2 = queue.top();
        const double p2 = queue.top_priority();

        if( (-p1) < mLoweRatio * (-p2) )
        {
            ret = j1;
        }
    }
    else if( queue.size() > 0 )
    {
        ret = queue.top();
    }

    //std::cout << "A " << ret << std::endl;

    if( ret >= 0 && mCheckEpipolar )
    {
        const int j = ret;

        ret = -1;

        Eigen::Vector3d pointi;
        pointi <<
            mUndistortedPoints[view][i].x,
            mUndistortedPoints[view][i].y,
            1.0;

        Eigen::Vector3d pointj;
        pointj <<
            mUndistortedPoints[other_view][j].x,
            mUndistortedPoints[other_view][j].y,
            1.0;

        Eigen::Vector3d line = mFundamentalMatrices[view] * pointi;

        const double l = line.head<2>().norm();

        if( l > 1.0e-7 )
        {
            line /= l;

            const double val = pointj.transpose() * line;

            //std::cout << std::fabs(val) << std::endl;

            if( std::fabs(val) < mEpipolarThreshold )
            {
                //ok = true;
                ret = j;
            }
        }
    }
    //std::cout << "B " << ret << std::endl;

    if( ret >= 0 && check_symmetry )
    {
        const int other = matchKeyPoint(f, other_view, ret, false);

        if(other != i)
        {
            ret = -1;
        }
    }
    //std::cout << "C " << ret << std::endl;

    return ret;
}

SLAMModuleResult SLAMModule1StereoMatcher::operator()()
{
    std::cout << "   STEREO MATCHING" << std::endl;

    SLAMReconstructionPtr reconstr = context()->reconstruction;

    if( reconstr->frames.empty() ) throw std::runtime_error("internal error");

    SLAMFramePtr f = reconstr->frames.back();

    // compute undistorted key points.

    for(int k=0; k<2; k++)
    {
        if( f->views[k].keypoints.empty() )
        {
            mUndistortedPoints[k].clear();
        }
        else
        {
            std::vector<cv::Point2f> tmp;

            cv::KeyPoint::convert( f->views[k].keypoints, tmp );

            cv::undistortPoints(
                tmp,
                mUndistortedPoints[k], 
                mStereoRigCalibration->cameras[k].calibration_matrix,
                mStereoRigCalibration->cameras[k].distortion_coefficients,
                cv::noArray(),
                mStereoRigCalibration->cameras[k].calibration_matrix);
        }
    }

    // proceed with matching.

    std::vector< std::pair<int,int> >& matches = f->stereo_matches;
    matches.clear();

    for(int i=0; i<f->views[0].keypoints.size(); i++)
    {
        const int j = matchKeyPoint(f, 0, i, mCheckSymmetry);

        if( j >= 0 )
        {
            matches.push_back( std::pair<int,int>(i, j) );
            //f->views[0].tracks[i].stereo_match = j;
            //f->views[1].tracks[j].stereo_match = i;
        }
    }

    mUndistortedPoints[0].clear();
    mUndistortedPoints[1].clear();

    if( context()->configuration->stereo_matcher.debug )
    {
        std::vector<cv::DMatch> matches2(matches.size());

        std::transform( matches.cbegin(), matches.cend(), matches2.begin(), [] ( const std::pair<int,int>& pair )
        {
            return cv::DMatch( pair.first, pair.second, 1.0 );
        });

        cv::Mat outimg;

        cv::drawMatches(
            f->views[0].image,
            f->views[0].keypoints,
            f->views[1].image,
            f->views[1].keypoints,
            matches2,
            outimg);

        context()->debug->saveImage(f->id, "STEREOMATCHING_matching.png", outimg);
    }

    std::cout << "      Number of stereo matches: " << f->stereo_matches.size() << std::endl;

    return SLAMModuleResult(false, SLAM_MODULE1_TRIANGULATION);
}

