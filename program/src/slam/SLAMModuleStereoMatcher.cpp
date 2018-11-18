#include <utility>
#include <opencv2/imgproc.hpp>
#include "SLAMModuleStereoMatcher.h"
#include "FinitePriorityQueue.h"
#include "Misc.h"

SLAMModuleStereoMatcher::SLAMModuleStereoMatcher(SLAMProjectPtr project) : SLAMModule(project)
{
    mCheckSymmetry = project->getParameterBoolean("stereo_matcher_check_lowe", true);
    mCheckLowe = project->getParameterBoolean("stereo_matcher_check_lowe", true);
    mCheckEpipolar = project->getParameterBoolean("stereo_matcher_check_epipolar", true);
    mEpipolarThreshold = project->getParameterReal("stereo_matcher_epipolar_threshold", 8.0);
    mLoweRatio = project->getParameterReal("stereo_matcher_lowe_ratio", 0.5);

    mCameraCalibration[0] = project->getLeftCameraCalibration();
    mCameraCalibration[1] = project->getRightCameraCalibration();
    mStereoRigCalibration = project->getStereoRigCalibration();
}

int SLAMModuleStereoMatcher::matchKeyPoint(FramePtr f, int view, int i, bool check_symmetry)
{
    const int other_view = (view + 1) % 2;

    FinitePriorityQueueF<int, double, 2> queue;

    for(int j=0; j<f->views[other_view].keypoints.size(); j++)
    {
        bool ok = true;

        if(ok && mCheckOctave)
        {
            ok = ( f->views[view].keypoints[i].octave == f->views[other_view].keypoints[j].octave );
        }

        if(ok && mCheckEpipolar)
        {
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

                ok = ( std::fabs(val) < mEpipolarThreshold );
            }
            else
            {
                ok = false;
            }
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

    if(ret >= 0 && check_symmetry )
    {
        const int other = matchKeyPoint(f, other_view, ret, false);

        if(other != i)
        {
            ret = -1;
        }
    }

    return ret;
}

void SLAMModuleStereoMatcher::match(FramePtr f, std::vector< std::pair<int,int> >& matches)
{
    // compute undistorted key points.

    {
        std::vector<cv::Point2f> tmp;

        for(int k=0; k<2; k++)
        {
            cv::KeyPoint::convert( f->views[k].keypoints, tmp );

            cv::undistortPoints(
                tmp,
                mUndistortedPoints[k], 
                mCameraCalibration[k]->calibration_matrix,
                mCameraCalibration[k]->distortion_coefficients,
                cv::noArray(),
                mCameraCalibration[k]->calibration_matrix);
        }
    }

    // compute fundamental matrices.

    mFundamentalMatrices[0] = Misc::computeFundamentalMatrix( mCameraCalibration[0], mCameraCalibration[1], mStereoRigCalibration );
    mFundamentalMatrices[1] = mFundamentalMatrices[0].transpose();

    // proceed with matching.

    matches.clear();

    for(int i=0; i<f->views[0].keypoints.size(); i++)
    {
        int j = matchKeyPoint(f, 0, i, mCheckSymmetry);

        if( j >= 0 )
        {
            matches.push_back( std::pair<int,int>(i, j) );
        }
    }
    std::cout << matches.size() << std::endl;

    mUndistortedPoints[0].clear();
    mUndistortedPoints[1].clear();
}

