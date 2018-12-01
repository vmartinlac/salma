#include <memory>
#include <random>
#include <iostream>
#include <Eigen/Eigen>
#include <sophus/se3.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <MVPnP.h>

int main(int num_args, char** args)
{
    std::vector<cv::Point3f> points;

    {
        std::default_random_engine en;
        std::uniform_real_distribution<double> rx(8.0, 10.0);
        std::uniform_real_distribution<double> ry(2.0, 4.0);
        std::uniform_real_distribution<double> rz(-1.0, 1.0);

        const int N = 40;

        for(int i=0; i<N; i++)
        {
            cv::Point3f pt( rx(en), ry(en), rz(en) );
            points.push_back(pt);
        }
    }

    cv::Mat K = ( cv::Mat_<double>(3, 3, CV_64F) << 500.0, 0.0, 500.0, 0.0, 500.0, 500.0, 0.0, 0.0, 1.0 );
    cv::Mat dist = cv::Mat_<double>(0, 0, CV_64F);

    Sophus::SE3d camera_to_rig;
    camera_to_rig.translation() << 3.0, 0.0, 0.0;

    std::vector<MVPnP::View> views(1);

    views[0].calibration_matrix = K;
    views[0].distortion_coefficients = dist;
    views[0].rig_to_camera = camera_to_rig.inverse();
    views[0].points = points;

    Eigen::Matrix3d tmp;
    tmp <<
        0.0, 0.0, 1.0,
        -1.0, 0.0, 0.0,
        0.0, -1.0, 0.0;

    Sophus::SE3d camera_to_world;
    camera_to_world.translation() << 2.0, 3.0, 0.0;
    camera_to_world.setRotationMatrix(tmp);

    /*
    camera_to_world.translation().setZero();
    camera_to_world.setQuaternion(Eigen::Quaterniond(Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitX())));
    std::cout << camera_to_world.log() << std::endl;
    exit(0);
    */

    cv::Mat rvec;
    cv::Mat tvec;
    cv::eigen2cv( camera_to_world.inverse().so3().log(), rvec );
    cv::eigen2cv( camera_to_world.inverse().translation(), tvec );

    cv::projectPoints(
        views[0].points,
        rvec,
        tvec,
        views[0].calibration_matrix,
        views[0].distortion_coefficients,
        views[0].projections);

    for(int i=0; i<views[0].points.size(); i++)
    {
        std::cout
            << views[0].points[i].x << ' '
            << views[0].points[i].y << ' '
            << views[0].points[i].z << ' '
            << views[0].projections[i].x << ' '
            << views[0].projections[i].y << std::endl;
    }

    std::vector< std::vector<bool> > inliers;

    Sophus::SE3d rig_to_world;
    rig_to_world.setRotationMatrix(tmp);
    rig_to_world.translation() << 2.0, 6.06, 0.0;
    //rig_to_world.translation() << 2.0, 6.00, 0.0;
    rig_to_world.setQuaternion( rig_to_world.unit_quaternion() * Eigen::Quaterniond(Eigen::AngleAxisd(0.01*M_PI, Eigen::Vector3d::UnitX())) );

    std::shared_ptr<MVPnP::Solver> s( MVPnP::Solver::create() );

    const bool ret = s->run(views, rig_to_world, false, inliers);

    std::cout << "Solver::run() returned " << ret << std::endl;
    std::cout << rig_to_world.translation().transpose() << std::endl;

    return 0;
}

