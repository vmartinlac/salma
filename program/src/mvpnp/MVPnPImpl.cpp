#include <Eigen/Eigen>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <iostream>
#include "MVPnPImpl.h"

MVPnP::SolverImpl::SolverImpl()
{
}

MVPnP::SolverImpl::~SolverImpl()
{
}

lbfgsfloatval_t MVPnP::SolverImpl::evaluateProc(
    void* data,
    const lbfgsfloatval_t* x,
    lbfgsfloatval_t* gradient,
    const int n,
    const lbfgsfloatval_t step)
{
    if( n != 7 ) throw std::runtime_error("internal error");

    SolverImpl* solver = static_cast<SolverImpl*>(data);

    lbfgsfloatval_t error = 0.0;
    std::fill( gradient, gradient+7, 0.0);
    int count = 0;

    for( const MVPnP::View& v : *solver->getViews() )
    {
        cv::Mat tvec;
        cv::Mat rvec;
        // TODO: compute tvec and rvec.
        Eigen::Matrix<double, 6, 7> Q;
        // TODO: compute Q.

        std::vector<cv::Point2f> projections;
        cv::Mat jac_cv;

        cv::projectPoints(v.points, rvec, tvec, v.calibration_matrix, v.distortion_coefficients, projections, jac_cv);

        Eigen::MatrixXd jac;
        cv::cv2eigen(jac_cv, jac);

        for( int i=0; i<v.points.size(); i++ )
        {
            const Eigen::Vector2d proj{ projections[i].x, projections[i].y };
            const Eigen::Vector2d proj_ref{ v.projections[i].x, v.projections[i].y };
            const Eigen::Vector2d delta_proj = proj - proj_ref;

            const Eigen::Matrix<double, 1, 7> grad = 2.0 * delta_proj.transpose() * jac.block<2,6>(2*i,0) * Q;

            for(int j=0; j<7; j++)
            {
                gradient[j] += grad(j);
            }

            error += delta_proj.squaredNorm();

            count++;
        }
    }

    // divide the error and the gradient by the number of points.

    error /= lbfgsfloatval_t(count);

    for(int i=0; i<7; i++)
    {
        gradient[i] /= lbfgsfloatval_t(count);
    }

    return error;
}

int MVPnP::SolverImpl::progressProc(
    void* data,
    const lbfgsfloatval_t* x,
    const lbfgsfloatval_t* gradient,
    const lbfgsfloatval_t fx,
    const lbfgsfloatval_t xnorm,
    const lbfgsfloatval_t gnorm,
    const lbfgsfloatval_t step,
    int n,
    int k,
    int ls)
{
    std::cout << "Multiview PnP L2 reprojection error = " << std::sqrt(fx) << std::endl;
    return 0;
}

bool MVPnP::SolverImpl::run( const std::vector<View>& views, Sophus::SE3d& rig_to_world, std::vector< std::vector<bool> >& inliers)
{
    mViews = &views;

    lbfgs_parameter_t params;
    lbfgsfloatval_t* x;
    lbfgsfloatval_t fx;

    {
        lbfgs_parameter_init(&params);

        Sophus::SE3d world_to_rig = rig_to_world.inverse();

        x = lbfgs_malloc(7);
        x[0] = world_to_rig.translation().x();
        x[1] = world_to_rig.translation().y();
        x[2] = world_to_rig.translation().z();
        x[3] = world_to_rig.unit_quaternion().x();
        x[4] = world_to_rig.unit_quaternion().y();
        x[5] = world_to_rig.unit_quaternion().z();
        x[6] = world_to_rig.unit_quaternion().w();

        fx = 0.0;
    }

    const int status = lbfgs(7, x, &fx, evaluateProc, progressProc, this, &params);

    if( status == LBFGS_SUCCESS )
    {
        Eigen::Vector3d world_to_rig_t;
        world_to_rig_t.x() = x[0];
        world_to_rig_t.y() = x[1];
        world_to_rig_t.z() = x[2];

        Eigen::Quaterniond world_to_rig_q;
        world_to_rig_q.x() = x[3];
        world_to_rig_q.y() = x[4];
        world_to_rig_q.z() = x[5];
        world_to_rig_q.w() = x[6];

        Sophus::SE3d world_to_rig;
        world_to_rig.translation() = world_to_rig_t;
        world_to_rig.setQuaternion(world_to_rig_q);

        rig_to_world = world_to_rig.inverse();
    }

    lbfgs_free(x);

    inliers.resize( views.size() );

    for( int i=0; i<views.size(); i++ )
    {
        inliers[i].assign(views[i].points.size(), true);
    }

    return ( status == LBFGS_SUCCESS );
}

const std::vector<MVPnP::View>* MVPnP::SolverImpl::getViews()
{
    return mViews;
}

/*
    void computeWorldToCamera(
        const Eigen::Vector3d& point_in_world,
        const Eigen::Vector3d& rig_to_camera_t,
        const Eigen::Matrix3d& rig_to_camera_R,
        const Eigen::Vector3d& world_to_rig_t,
        const Eigen::Quaternio,
        Eigen::Vector2d& point_in_camera,
        Eigen::Matrix<double, >& jac_wrt_);

    void projection(
        const cv::Mat& calibration_matrix,
        const cv::Mat& distortion_coefficients,
        const Eigen::Vector3d& point,
        Eigen::Vector2d& projection,
        Eigen::Matrix<double, >& jac_wrt_point);
*/

