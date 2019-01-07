#include <opencv2/calib3d.hpp>
#include "MVPnP.h"
#include "MVPnPRANSACLM.h"
#include "MVPnPLM.h"
#include "MVPnPMonoOpenCV.h"

MVPnP::Solver::Solver()
{
}

MVPnP::Solver::~Solver()
{
}

int MVPnP::Solver::extractSelection(
    const std::vector<View>& views,
    const std::vector< std::vector<bool> >& selection,
    std::vector<View>& new_views)
{
    if( selection.size() != views.size() ) throw std::runtime_error("internal error");

    int count;

    new_views.resize(views.size());

    for(int i=0; i<views.size(); i++)
    {
        if( views[i].points.size() != selection[i].size() ) throw std::runtime_error("internal error");

        new_views[i].calibration_matrix = views[i].calibration_matrix;
        new_views[i].distortion_coefficients = views[i].distortion_coefficients;
        new_views[i].rig_to_camera = views[i].rig_to_camera;

        for(int j=0; j<views[i].points.size(); j++)
        {
            if( selection[i][j] )
            {
                new_views[i].points.push_back( views[i].points[j] );
                new_views[i].projections.push_back( views[i].projections[j] );
                count++;
            }
        }
    }

    return count;
}

int MVPnP::Solver::findInliers(
    const std::vector<View>& views,
    const Sophus::SE3d& rig_to_world,
    double threshold,
    std::vector< std::vector<bool> >& inliers)
{
    int inlier_count = 0;

    inliers.clear();
    inliers.resize( views.size() );

    for(int i=0; i<views.size(); i++)
    {
        const int N = views[i].points.size();

        std::vector<cv::Point3f> to_project(N);

        const Sophus::SE3d world_to_camera = views[i].rig_to_camera * rig_to_world.inverse();

        for(int j=0; j<N; j++)
        {
            Eigen::Vector3d in_world_frame;
            in_world_frame.x() = views[i].points[j].x;
            in_world_frame.y() = views[i].points[j].y;
            in_world_frame.z() = views[i].points[j].z;

            Eigen::Vector3d in_camera_frame = world_to_camera * in_world_frame;

            to_project[j].x = in_camera_frame.x();
            to_project[j].y = in_camera_frame.y();
            to_project[j].z = in_camera_frame.z();
        }

        std::vector<cv::Point2f> projected;
        if( to_project.empty() == false )
        {
            cv::projectPoints(
                to_project,
                cv::Mat::zeros(3,1,CV_64F),
                cv::Mat::zeros(3,1,CV_64F),
                views[i].calibration_matrix,
                views[i].distortion_coefficients,
                projected);
        }

        if(projected.size() != N) throw std::runtime_error("internal error");

        inliers[i].resize(N);

        for(int j=0; j<N; j++)
        {
            const double distance = cv::norm(projected[j] - views[i].projections[j]);

            inliers[i][j] = (distance < threshold);

            if( inliers[i][j] )
            {
                inlier_count++;
            }
        }
    }

    return inlier_count;
}

