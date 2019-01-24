#include <set>
#include <iomanip>
#include <fstream>
#include <iostream>
#include <map>
#include <Eigen/Eigen>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include "SLAMModuleEKF.h"
#include "SLAMMath.h"

SLAMModuleEKF::SLAMModuleEKF(SLAMContextPtr con) :
    SLAMModule(SLAM_MODULE2_EKF, con)
{
    mLastFrameId = -1;
    mLastFrameTimestamp = 0.0;
}

SLAMModuleEKF::~SLAMModuleEKF()
{
}

bool SLAMModuleEKF::init()
{
    return true;
}

SLAMModuleResult SLAMModuleEKF::operator()()
{
    std::cout << "   EXTENDED KALMAN FILTER" << std::endl;

    SLAMReconstructionPtr reconstr = context()->reconstruction;

    if( reconstr->frames.empty() ) throw std::runtime_error("internal error");

    mCurrentFrame = reconstr->frames.back();

    if(mCurrentFrame->aligned_wrt_previous_frame)
    {
        prepareLocalMap();
        ekfPrediction();
        ekfUpdate();
    }
    else
    {
        initializeState();
    }

    exportResult();

    std::cout << "      Local map size: " << mLocalMap.size() << std::endl;

    std::cout << "      Position-x: " << mMu(0) << std::endl;
    std::cout << "      Position-y: " << mMu(1) << std::endl;
    std::cout << "      Position-z: " << mMu(2) << std::endl;
    std::cout << "      Position-x sdev: " << std::sqrt(mSigma(0,0)) << std::endl;
    std::cout << "      Position-y sdev: " << std::sqrt(mSigma(1,1)) << std::endl;
    std::cout << "      Position-z sdev: " << std::sqrt(mSigma(2,2)) << std::endl;

    std::cout << "      Attitude-x: " << mMu(3) << std::endl;
    std::cout << "      Attitude-y: " << mMu(4) << std::endl;
    std::cout << "      Attitude-z: " << mMu(5) << std::endl;
    std::cout << "      Attitude-w: " << mMu(6) << std::endl;
    std::cout << "      Attitude-x sdev: " << std::sqrt(mSigma(3,3)) << std::endl;
    std::cout << "      Attitude-y sdev: " << std::sqrt(mSigma(4,4)) << std::endl;
    std::cout << "      Attitude-z sdev: " << std::sqrt(mSigma(5,5)) << std::endl;
    std::cout << "      Attitude-w sdev: " << std::sqrt(mSigma(6,6)) << std::endl;

    std::cout << "      Linear-velocity-x: " << mMu(7) << std::endl;
    std::cout << "      Linear-velocity-y: " << mMu(8) << std::endl;
    std::cout << "      Linear-velocity-z: " << mMu(9) << std::endl;
    std::cout << "      Linear-velocity-x sdev: " << std::sqrt(mSigma(7,7)) << std::endl;
    std::cout << "      Linear-velocity-y sdev: " << std::sqrt(mSigma(8,8)) << std::endl;
    std::cout << "      Linear-velocity-z sdev: " << std::sqrt(mSigma(9,9)) << std::endl;

    std::cout << "      Angular-velocity-x: " << mMu(10) << std::endl;
    std::cout << "      Angular-velocity-y: " << mMu(11) << std::endl;
    std::cout << "      Angular-velocity-z: " << mMu(12) << std::endl;
    std::cout << "      Angular-velocity-x sdev: " << std::sqrt(mSigma(10,10)) << std::endl;
    std::cout << "      Angular-velocity-y sdev: " << std::sqrt(mSigma(11,11)) << std::endl;
    std::cout << "      Angular-velocity-z sdev: " << std::sqrt(mSigma(12,12)) << std::endl;

    mLastFrameId = mCurrentFrame->id;
    mLastFrameTimestamp = mCurrentFrame->timestamp;
    mCurrentFrame.reset();

    return SLAMModuleResult(true, SLAM_MODULE2_OPTICALFLOW);
}

void SLAMModuleEKF::initializeState()
{
    mLocalMap.clear();

    mMu.resize(13);
    mMu.setZero();
    mMu(0) = mCurrentFrame->frame_to_world.translation().x();
    mMu(1) = mCurrentFrame->frame_to_world.translation().y();
    mMu(2) = mCurrentFrame->frame_to_world.translation().z();
    mMu(3) = mCurrentFrame->frame_to_world.unit_quaternion().x();
    mMu(4) = mCurrentFrame->frame_to_world.unit_quaternion().y();
    mMu(5) = mCurrentFrame->frame_to_world.unit_quaternion().z();
    mMu(6) = mCurrentFrame->frame_to_world.unit_quaternion().w();
    mMu(7) = 0.0;
    mMu(8) = 0.0;
    mMu(9) = 0.0;
    mMu(10) = 0.0;
    mMu(11) = 0.0;
    mMu(12) = 0.0;

    const double position_sdev = context()->configuration->ekf.initial_position_sdev;
    const double attitude_sdev = context()->configuration->ekf.initial_attitude_sdev;
    const double linear_momentum_sdev = context()->configuration->ekf.initial_linear_velocity_sdev;
    const double angular_momentum_sdev = context()->configuration->ekf.initial_angular_velocity_sdev;

    mSigma.resize(13, 13);
    mSigma.setZero();
    mSigma.diagonal().segment<3>(0).fill(position_sdev*position_sdev);
    mSigma.diagonal().segment<4>(3).fill(attitude_sdev*attitude_sdev);
    mSigma.diagonal().segment<3>(7).fill(linear_momentum_sdev*linear_momentum_sdev);
    mSigma.diagonal().segment<3>(10).fill(angular_momentum_sdev*angular_momentum_sdev);
}

void SLAMModuleEKF::prepareLocalMap()
{
    std::vector<SLAMMapPointPtr> new_local_map;
    Eigen::VectorXd new_mu;
    Eigen::MatrixXd new_sigma;

    // create new local map with as many mappoints as possible.

    {
        std::set<int> new_local_map_content;

        new_local_map.resize(mLocalMap.size());

        for(int i=0; i<mLocalMap.size(); i++)
        {
            if( new_local_map_content.count( mLocalMap[i]->id ) != 0 ) throw std::runtime_error("internal error");

            new_local_map[i] = mLocalMap[i];
            new_local_map_content.insert(mLocalMap[i]->id);
        }

        for(int i=0; i<2; i++)
        {
            for(SLAMTrack& t : mCurrentFrame->views[i].tracks)
            {
                if( t.mappoint && t.mappoint->frame_id_of_last_position_update == mLastFrameId && new_local_map_content.count(t.mappoint->id) == 0 )
                {
                    new_local_map.push_back(t.mappoint);
                    new_local_map_content.insert(t.mappoint->id);
                }
            }
        }
    }

    // if there are too many points in local map, remove some.

    const int max_local_map_size = context()->configuration->ekf.max_local_map_size;

    if( new_local_map.size() > max_local_map_size )
    {
        auto comp = [] (SLAMMapPointPtr P1, SLAMMapPointPtr P2)
        {
            // TODO: is this criteria appropriate?
            return (P1->frame_id_of_creation < P2->frame_id_of_creation);
        };

        std::sort(new_local_map.begin(), new_local_map.end(), comp);

        while( new_local_map.size() > max_local_map_size)
        {
            new_local_map.pop_back();
        }
    }

    // create new mu and sigma.

    {
        std::map<int,int> prev_local_map_inv;

        for(int i=0; i<mLocalMap.size(); i++)
        {
            prev_local_map_inv[mLocalMap[i]->id] = i;
        }

        const int new_dimension = 13 + 3*new_local_map.size();

        new_mu.resize(new_dimension);
        new_mu.setZero();

        new_sigma.resize(new_dimension, new_dimension);
        new_sigma.setZero();

        new_mu.head<13>() = mMu.head<13>();
        new_sigma.block<13,13>(0,0) = mSigma.block<13,13>(0,0);

        for(int i=0; i<new_local_map.size(); i++)
        {
            std::map<int,int>::iterator it = prev_local_map_inv.find(new_local_map[i]->id);
            
            if(it == prev_local_map_inv.end())
            {
                if( new_local_map[i]->frame_id_of_last_position_update == mLastFrameId )
                {
                    new_mu.segment<3>(13 + i*3) = new_local_map[i]->position;

                    new_sigma.block<3,3>(13 + i*3, 13 + i*3) = new_local_map[i]->position_covariance.block<3,3>(0,0); // covariance of mappoint_position wrt mappoint_position.
                    new_sigma.block<3,3>(13 + i*3, 0) = new_local_map[i]->position_covariance.block<3,3>(0,3); // covariance of mappoint_position wrt camera position.
                    new_sigma.block<3,4>(13 + i*3, 3) = new_local_map[i]->position_covariance.block<3,4>(0,6); // covariance of mappoint_position wrt camera attitude
                    new_sigma.block<3,3>(0, 13 + i*3) = new_local_map[i]->position_covariance.block<3,3>(0,3).transpose(); // covariance of mappoint_position wrt camera position.
                    new_sigma.block<4,3>(3, 13 + i*3) = new_local_map[i]->position_covariance.block<3,4>(0,6).transpose(); // covariance of mappoint_position wrt camera attitude
                }
                else
                {
                    throw std::runtime_error("internal error");
                }
            }
            else
            {
                new_mu.segment<3>(13 + i*3) = mMu.segment<3>(13 + 3*it->second);

                new_sigma.block<13, 3>(0, 13 + 3*i) = mSigma.block<13, 3>(0, 13+3*it->second);
                new_sigma.block<3, 13>(13 + 3*i, 0) = mSigma.block<3, 13>(13+3*it->second, 0);

                for(int j=0; j<new_local_map.size(); j++)
                {
                    std::map<int,int>::iterator it2 = prev_local_map_inv.find(new_local_map[j]->id);

                    if( it2 != prev_local_map_inv.end() )
                    {
                        new_sigma.block<3,3>(13+3*i, 13+3*j) = mSigma.block<3,3>(13+3*it->second, 13+3*it2->second);
                        new_sigma.block<3,3>(13+3*j, 13+3*i) = mSigma.block<3,3>(13+3*it2->second, 13+3*it->second);
                    }
                }
            }
        }

        // assert that new_sigma is symmetric.
        //const double symm_err = ( new_sigma - new_sigma.transpose() ).norm();
        //std::cout << symm_err << std::endl;
    }

    mLocalMap.swap(new_local_map);
    mMu.swap(new_mu);
    mSigma.swap(new_sigma);

    /*
    std::ofstream f("tmp"+std::to_string(mCurrentFrame->id)+".csv", std::ofstream::app);
    f << "mu " << mCurrentFrame->id << std::endl;
    f << mMu << std::endl;
    f << "sigma " << mCurrentFrame->id << std::endl;
    for(int i=0; i<mSigma.rows(); i++)
    {
        for(int j=0; j<mSigma.cols(); j++)
        {
            f << std::setfill('0') << std::setw(12) << mSigma(i,j) << ' ';
        }
        f << std::endl;
    }
    f.close();
    if(mCurrentFrame->id >= 3) exit(0);
    */
}

void SLAMModuleEKF::ekfPrediction()
{
    /*
    {

    Eigen::VectorXd X(13);
    X.segment<3>(0) << 0.0, 0.0, 0.0;
    X.segment<4>(3) << 0.0, 0.0, 0.0, 1.0;
    X.segment<3>(7) << 0.0, 0.0, 0.0;
    X.segment<3>(10) << 0.0, 0.0, M_PI*0.5;

    for(int i=0; i<6; i++)
    {
        Eigen::VectorXd f;
        Eigen::SparseMatrix<double> J;

        compute_f(X, 0.5, f, J);
        std::cout << f.transpose() << std::endl;
        X.swap(f);
    }
    exit(0);

    }
    */

    Eigen::SparseMatrix<double> Q;
    Eigen::SparseMatrix<double> J;

    Eigen::VectorXd new_mu;
    Eigen::MatrixXd new_sigma;

    const double dt = mCurrentFrame->timestamp - mLastFrameTimestamp;

    const int num_landmarks = mLocalMap.size();

    const int dim = 13 + 3*num_landmarks;

    // set Q.
    {
        const double sigma_v = dt * context()->configuration->ekf.prediction_linear_acceleration_sdev;
        const double sigma_w = dt * context()->configuration->ekf.prediction_angular_acceleration_sdev;

        Q.resize(dim, dim);
        Q.setZero();
        Q.reserve(6);
        Q.insert(7,7) = sigma_v*sigma_v;
        Q.insert(8,8) = sigma_v*sigma_v;
        Q.insert(9,9) = sigma_v*sigma_v;
        Q.insert(10,10) = sigma_w*sigma_w;
        Q.insert(11,11) = sigma_w*sigma_w;
        Q.insert(12,12) = sigma_w*sigma_w;
        Q.makeCompressed();
    }

    compute_f(mMu, dt, new_mu, J);

    new_sigma = J * (mSigma + Q) * J.transpose();

    new_mu.segment<4>(3).normalize(); // normalize attitude.

    //const double symm_err = ( new_sigma - new_sigma.transpose() ).norm();
    //std::cout << "symm = " << symm_err << std::endl;

    /*
    std::cout << mMu.head<13>().transpose() << std::endl;
    std::cout << new_mu.head<13>().transpose() << std::endl;
    std::cout << mSigma.block<13,13>(0,0) << std::endl;
    std::cout << new_sigma.block<13,13>(0,0) << std::endl;
    */

    mMu.swap(new_mu);
    mSigma.swap(new_sigma);
}

void SLAMModuleEKF::ekfUpdate()
{
    std::vector<VisiblePoint> visible_points;

    // find visible points (mappoints which are in local map and have at least one projection in current frame).

    {
        std::map<int,int> local_map_inv;

        for(int i=0; i<mLocalMap.size(); i++)
        {
            local_map_inv[mLocalMap[i]->id] = i;
        }


        for(int i=0; i<2; i++)
        {
            for(int j=0; j<mCurrentFrame->views[i].keypoints.size(); j++)
            {
                SLAMMapPointPtr mp = mCurrentFrame->views[i].tracks[j].mappoint;

                if(mp)
                {
                    std::map<int,int>::iterator it = local_map_inv.find(mp->id);

                    if(it != local_map_inv.end())
                    {
                        VisiblePoint vpt;
                        vpt.local_index = it->second;
                        vpt.view = i;
                        vpt.keypoint = j;
                        visible_points.push_back(vpt);
                    }
                }
            }
        }
    }


    if(visible_points.empty())
    {
        std::cout << "      NO EKF UPDATE BECAUSE!" << std::endl;
    }
    else
    {
        Eigen::VectorXd predicted_projections;
        Eigen::VectorXd observed_projections;
        Eigen::SparseMatrix<double> J;
        Eigen::SparseMatrix<double> observation_noise;

        // compute predicted and observed projections.
        {
            const int dim = 2*visible_points.size();

            compute_h(mMu, visible_points, predicted_projections, J);

            if(predicted_projections.size() != dim) throw std::runtime_error("internal error");

            observation_noise.setZero();
            observation_noise.resize(dim, dim);
            observation_noise.reserve(dim);

            observed_projections.resize(dim);

            const double proj_sdev = context()->configuration->ekf.update_projection_sdev;

            for(int i=0; i<visible_points.size(); i++)
            {
                observed_projections(2*i+0) = mCurrentFrame->views[visible_points[i].view].keypoints[visible_points[i].keypoint].pt.x;
                observed_projections(2*i+1) = mCurrentFrame->views[visible_points[i].view].keypoints[visible_points[i].keypoint].pt.y;

                observation_noise.insert(2*i+0, 2*i+0) = proj_sdev*proj_sdev;
                observation_noise.insert(2*i+1, 2*i+1) = proj_sdev*proj_sdev;
            }
        }

        // compute new belief.

        {
            const Eigen::VectorXd residuals = observed_projections - predicted_projections;

            const Eigen::MatrixXd S = J * mSigma * J.transpose() + observation_noise;

            Eigen::LDLT< Eigen::MatrixXd > solver;
            solver.compute( S );

            Eigen::VectorXd new_mu = mMu + mSigma * J.transpose() * solver.solve( residuals );
            Eigen::MatrixXd new_sigma = mSigma - mSigma * J.transpose() * solver.solve( J * mSigma );

            new_mu.segment<4>(3).normalize(); // normalize attitude.

            mMu.swap(new_mu);
            mSigma.swap(new_sigma);
        }
    }
}

void SLAMModuleEKF::exportResult()
{
    // export frame pose.

    {
        mCurrentFrame->frame_to_world.translation() = mMu.segment<3>(0);

        mCurrentFrame->frame_to_world.setQuaternion(SLAMMath::convert(mMu.segment<4>(3)));

        mCurrentFrame->pose_covariance = mSigma.block<7,7>(0,0);
    }

    // export mappoints positions.
    
    {
        for(int i=0; i<mLocalMap.size(); i++)
        {
            mLocalMap[i]->position = mMu.segment<3>(13+3*i);

            mLocalMap[i]->position_covariance.block<3,3>(0,0) = mSigma.block<3,3>(13+3*i, 13+3*i); // covariance of mappoint_position wrt mappoint_position.
            mLocalMap[i]->position_covariance.block<3,3>(0,3) = mSigma.block<3,3>(13+3*i, 0); // covariance of mappoint_position wrt camera position.
            mLocalMap[i]->position_covariance.block<3,4>(0,6) = mSigma.block<3,4>(13+3*i, 3); // covariance of mappoint_position wrt camera attitude

            mLocalMap[i]->frame_id_of_last_position_update = mCurrentFrame->id;
        }
    }
}

void SLAMModuleEKF::compute_f(
    const Eigen::VectorXd& X,
    double dt,
    Eigen::VectorXd& f,
    Eigen::SparseMatrix<double>& J)
{
    const int dim = X.size();

    if( (dim - 13) % 3 != 0 ) throw std::runtime_error("internal error");

    const int num_landmarks = (dim - 13)/3;

    const Eigen::Vector3d r = X.segment<3>(0);
    const Eigen::Vector4d q = X.segment<4>(3);
    const Eigen::Vector3d v = X.segment<3>(7);
    const Eigen::Vector3d w = X.segment<3>(10);

    Eigen::Matrix<double, 4, 3> Js;
    const Eigen::Vector4d s = rotationVectorToQuaternion(w*dt, Js);

    Eigen::Matrix<double, 4, 8> Jt;
    Eigen::Vector4d t;
    SLAMMath::computeQuaternionQuaternionProduct(q, s, &t, &Jt);

    const Eigen::Matrix4d Jq_wrt_q = Jt.leftCols<4>();
    const Eigen::Matrix<double, 4, 3> Jq_wrt_w = Jt.rightCols<4>() * Js;

    // fill f.

    f.resize(dim);
    f.segment<3>(0) = r + dt*v;
    f.segment<4>(3) = t.normalized();
    f.segment<3>(7) = v;
    f.segment<3>(10) = w;
    f.tail(dim-13) = X.tail(dim-13);

    // fill J.

    Eigen::SparseMatrix<double, Eigen::RowMajor> Jrm;

    Jrm.resize(dim, dim);
    Jrm.setZero();
    Jrm.reserve(40 + 3*num_landmarks);

    // position.

    Jrm.insert(0,0) = 1.0;
    Jrm.insert(0,7) = dt;
    Jrm.insert(1,1) = 1.0;
    Jrm.insert(1,8) = dt;
    Jrm.insert(2,2) = 1.0;
    Jrm.insert(2,9) = dt;

    // attitude.

    Jrm.insert(3, 3) = Jq_wrt_q(0,0);
    Jrm.insert(3, 4) = Jq_wrt_q(0,1);
    Jrm.insert(3, 5) = Jq_wrt_q(0,2);
    Jrm.insert(3, 6) = Jq_wrt_q(0,3);
    Jrm.insert(3, 10) = Jq_wrt_w(0,0);
    Jrm.insert(3, 11) = Jq_wrt_w(0,1);
    Jrm.insert(3, 12) = Jq_wrt_w(0,2);

    Jrm.insert(4, 3) = Jq_wrt_q(1,0);
    Jrm.insert(4, 4) = Jq_wrt_q(1,1);
    Jrm.insert(4, 5) = Jq_wrt_q(1,2);
    Jrm.insert(4, 6) = Jq_wrt_q(1,3);
    Jrm.insert(4, 10) = Jq_wrt_w(1,0);
    Jrm.insert(4, 11) = Jq_wrt_w(1,1);
    Jrm.insert(4, 12) = Jq_wrt_w(1,2);

    Jrm.insert(5, 3) = Jq_wrt_q(2,0);
    Jrm.insert(5, 4) = Jq_wrt_q(2,1);
    Jrm.insert(5, 5) = Jq_wrt_q(2,2);
    Jrm.insert(5, 6) = Jq_wrt_q(2,3);
    Jrm.insert(5, 10) = Jq_wrt_w(2,0);
    Jrm.insert(5, 11) = Jq_wrt_w(2,1);
    Jrm.insert(5, 12) = Jq_wrt_w(2,2);

    Jrm.insert(6, 3) = Jq_wrt_q(3,0);
    Jrm.insert(6, 4) = Jq_wrt_q(3,1);
    Jrm.insert(6, 5) = Jq_wrt_q(3,2);
    Jrm.insert(6, 6) = Jq_wrt_q(3,3);
    Jrm.insert(6, 10) = Jq_wrt_w(3,0);
    Jrm.insert(6, 11) = Jq_wrt_w(3,1);
    Jrm.insert(6, 12) = Jq_wrt_w(3,2);

    // linear velocity.

    Jrm.insert(7, 7) = 1.0;
    Jrm.insert(8, 8) = 1.0;
    Jrm.insert(9, 9) = 1.0;

    // angular velocity.

    Jrm.insert(10, 10) = 1.0;
    Jrm.insert(11, 11) = 1.0;
    Jrm.insert(12, 12) = 1.0;

    for(int i=0; i<num_landmarks; i++)
    {
        Jrm.insert(13+3*i+0, 13+3*i+0) = 1.0;
        Jrm.insert(13+3*i+1, 13+3*i+1) = 1.0;
        Jrm.insert(13+3*i+2, 13+3*i+2) = 1.0;
    }

    Jrm.makeCompressed();

    J = Jrm;
}

Eigen::Vector4d SLAMModuleEKF::rotationVectorToQuaternion(const Eigen::Vector3d& v, Eigen::Matrix<double, 4, 3>& J)
{
    // TODO: optimize the evaluation of these sympy-generated formulae.

    const double wx0 = v.x();
    const double wy0 = v.y();
    const double wz0 = v.z();

    const double n0 = std::sqrt(wx0*wx0 + wy0*wy0 + wz0*wz0);

    double wx = 0.0;
    double wy = 0.0;
    double wz = 0.0;

    Eigen::Vector4d ret;

    const double threshold = 5.0e-6;

    if(n0 >= threshold)
    {
        wx = wx0;
        wy = wy0;
        wz = wz0;

        ret(0) = wx*pow(pow(wx, 2) + pow(wy, 2) + pow(wz, 2), -0.5)*sin((1.0/2.0)*sqrt(pow(wx, 2) + pow(wy, 2) + pow(wz, 2)));
        ret(1) = wy*pow(pow(wx, 2) + pow(wy, 2) + pow(wz, 2), -0.5)*sin((1.0/2.0)*sqrt(pow(wx, 2) + pow(wy, 2) + pow(wz, 2)));
        ret(2) = wz*pow(pow(wx, 2) + pow(wy, 2) + pow(wz, 2), -0.5)*sin((1.0/2.0)*sqrt(pow(wx, 2) + pow(wy, 2) + pow(wz, 2)));
        ret(3) = cos((1.0/2.0)*sqrt(pow(wx, 2) + pow(wy, 2) + pow(wz, 2)));
    }
    else
    {
        ret(0) = 0.0;
        ret(1) = 0.0;
        ret(2) = 0.0;
        ret(3) = 1.0;

        std::normal_distribution<double> normal;

        double alpha = 0.0;

        do
        {
            wx = normal(mEngine);
            wy = normal(mEngine);
            wz = normal(mEngine);
            alpha = std::sqrt(wx*wx + wy*wy + wz*wz);
        }
        while(alpha < threshold);

        wx *= threshold / alpha;
        wy *= threshold / alpha;
        wz *= threshold / alpha;
    }

    J( 0, 0 ) = -1.0*pow(wx, 2)*pow(pow(wx, 2) + pow(wy, 2) + pow(wz, 2), -1.5)*sin((1.0/2.0)*sqrt(pow(wx, 2) + pow(wy, 2) + pow(wz, 2))) + 0.5*pow(wx, 2)*1.0/(pow(wx, 2) + pow(wy, 2) + pow(wz, 2))*cos((1.0/2.0)*sqrt(pow(wx, 2) + pow(wy, 2) + pow(wz, 2))) + pow(pow(wx, 2) + pow(wy, 2) + pow(wz, 2), -0.5)*sin((1.0/2.0)*sqrt(pow(wx, 2) + pow(wy, 2) + pow(wz, 2)));
    J( 0, 1 ) = -1.0*wx*wy*pow(pow(wx, 2) + pow(wy, 2) + pow(wz, 2), -1.5)*sin((1.0/2.0)*sqrt(pow(wx, 2) + pow(wy, 2) + pow(wz, 2))) + 0.5*wx*wy*1.0/(pow(wx, 2) + pow(wy, 2) + pow(wz, 2))*cos((1.0/2.0)*sqrt(pow(wx, 2) + pow(wy, 2) + pow(wz, 2)));
    J( 0, 2 ) = -1.0*wx*wz*pow(pow(wx, 2) + pow(wy, 2) + pow(wz, 2), -1.5)*sin((1.0/2.0)*sqrt(pow(wx, 2) + pow(wy, 2) + pow(wz, 2))) + 0.5*wx*wz*1.0/(pow(wx, 2) + pow(wy, 2) + pow(wz, 2))*cos((1.0/2.0)*sqrt(pow(wx, 2) + pow(wy, 2) + pow(wz, 2)));
    J( 1, 0 ) = -1.0*wx*wy*pow(pow(wx, 2) + pow(wy, 2) + pow(wz, 2), -1.5)*sin((1.0/2.0)*sqrt(pow(wx, 2) + pow(wy, 2) + pow(wz, 2))) + 0.5*wx*wy*1.0/(pow(wx, 2) + pow(wy, 2) + pow(wz, 2))*cos((1.0/2.0)*sqrt(pow(wx, 2) + pow(wy, 2) + pow(wz, 2)));
    J( 1, 1 ) = -1.0*pow(wy, 2)*pow(pow(wx, 2) + pow(wy, 2) + pow(wz, 2), -1.5)*sin((1.0/2.0)*sqrt(pow(wx, 2) + pow(wy, 2) + pow(wz, 2))) + 0.5*pow(wy, 2)*1.0/(pow(wx, 2) + pow(wy, 2) + pow(wz, 2))*cos((1.0/2.0)*sqrt(pow(wx, 2) + pow(wy, 2) + pow(wz, 2))) + pow(pow(wx, 2) + pow(wy, 2) + pow(wz, 2), -0.5)*sin((1.0/2.0)*sqrt(pow(wx, 2) + pow(wy, 2) + pow(wz, 2)));
    J( 1, 2 ) = -1.0*wy*wz*pow(pow(wx, 2) + pow(wy, 2) + pow(wz, 2), -1.5)*sin((1.0/2.0)*sqrt(pow(wx, 2) + pow(wy, 2) + pow(wz, 2))) + 0.5*wy*wz*1.0/(pow(wx, 2) + pow(wy, 2) + pow(wz, 2))*cos((1.0/2.0)*sqrt(pow(wx, 2) + pow(wy, 2) + pow(wz, 2)));
    J( 2, 0 ) = -1.0*wx*wz*pow(pow(wx, 2) + pow(wy, 2) + pow(wz, 2), -1.5)*sin((1.0/2.0)*sqrt(pow(wx, 2) + pow(wy, 2) + pow(wz, 2))) + 0.5*wx*wz*1.0/(pow(wx, 2) + pow(wy, 2) + pow(wz, 2))*cos((1.0/2.0)*sqrt(pow(wx, 2) + pow(wy, 2) + pow(wz, 2)));
    J( 2, 1 ) = -1.0*wy*wz*pow(pow(wx, 2) + pow(wy, 2) + pow(wz, 2), -1.5)*sin((1.0/2.0)*sqrt(pow(wx, 2) + pow(wy, 2) + pow(wz, 2))) + 0.5*wy*wz*1.0/(pow(wx, 2) + pow(wy, 2) + pow(wz, 2))*cos((1.0/2.0)*sqrt(pow(wx, 2) + pow(wy, 2) + pow(wz, 2)));
    J( 2, 2 ) = -1.0*pow(wz, 2)*pow(pow(wx, 2) + pow(wy, 2) + pow(wz, 2), -1.5)*sin((1.0/2.0)*sqrt(pow(wx, 2) + pow(wy, 2) + pow(wz, 2))) + 0.5*pow(wz, 2)*1.0/(pow(wx, 2) + pow(wy, 2) + pow(wz, 2))*cos((1.0/2.0)*sqrt(pow(wx, 2) + pow(wy, 2) + pow(wz, 2))) + pow(pow(wx, 2) + pow(wy, 2) + pow(wz, 2), -0.5)*sin((1.0/2.0)*sqrt(pow(wx, 2) + pow(wy, 2) + pow(wz, 2)));
    J( 3, 0 ) = -0.5*wx*pow(pow(wx, 2) + pow(wy, 2) + pow(wz, 2), -0.5)*sin((1.0/2.0)*sqrt(pow(wx, 2) + pow(wy, 2) + pow(wz, 2)));
    J( 3, 1 ) = -0.5*wy*pow(pow(wx, 2) + pow(wy, 2) + pow(wz, 2), -0.5)*sin((1.0/2.0)*sqrt(pow(wx, 2) + pow(wy, 2) + pow(wz, 2)));
    J( 3, 2 ) = -0.5*wz*pow(pow(wx, 2) + pow(wy, 2) + pow(wz, 2), -0.5)*sin((1.0/2.0)*sqrt(pow(wx, 2) + pow(wy, 2) + pow(wz, 2)));

    return ret;
}

void SLAMModuleEKF::compute_h(
    const Eigen::VectorXd& X,
    const std::vector<VisiblePoint>& visible_points,
    Eigen::VectorXd& h,
    Eigen::SparseMatrix<double>& J)
{
    StereoRigCalibrationDataPtr calibration = context()->calibration;;

    SLAMFramePtr frame = mCurrentFrame;

    const int dim = 2 * visible_points.size();

    const Eigen::Vector3d rig_to_world_t = X.segment<3>(0);

    const Eigen::Quaterniond rig_to_world_q( X(6), X(3), X(4), X(5) );

    const Sophus::SE3d rig_to_world(rig_to_world_q, rig_to_world_t);

    Eigen::Matrix3d rig_to_camera_R[2];

    Sophus::SE3d world_to_camera[2];

    for(int i=0; i<2; i++)
    {
        rig_to_camera_R[i] = calibration->cameras[i].camera_to_rig.rotationMatrix().transpose();
        world_to_camera[i] = calibration->cameras[i].camera_to_rig.inverse() * rig_to_world.inverse();
    }

    Eigen::Matrix<double, 9, 4> B;
    jacobianOfQuaternionToRotationMatrix(rig_to_world_q.inverse(), B);

    // jacobian of transposition of quaternion.
    Eigen::Matrix4d C;
    C.setZero();
    C.diagonal() << -1.0, -1.0, -1.0, 1.0;

    h.resize(dim);

    J.setZero();
    J.resize(dim, 13 + 3 * mLocalMap.size());
    J.reserve( 20 * visible_points.size() );

    for(int i=0; i<visible_points.size(); i++)
    {
        const VisiblePoint& vpt = visible_points[i];

        SLAMMapPointPtr mpt = mLocalMap[vpt.local_index];
        SLAMMapPointPtr mpt2 = frame->views[vpt.view].tracks[vpt.keypoint].mappoint;

        if( bool(mpt) == false || bool(mpt2) == false || mpt->id != mpt2->id ) throw std::runtime_error("internal error");

        const Eigen::Vector3d mappoint_in_camera_frame = world_to_camera[vpt.view] * mpt->position;

        const std::vector<cv::Point3f> mappoint_in_camera_frame_cv{ cv::Point3f(mappoint_in_camera_frame.x(), mappoint_in_camera_frame.y(), mappoint_in_camera_frame.z()) };

        std::vector<cv::Point2f> proj;
        cv::Mat J_proj_cv;

        cv::projectPoints(
            mappoint_in_camera_frame_cv,
            cv::Mat::zeros(3, 1, CV_64F),
            cv::Mat::zeros(3, 1, CV_64F),
            calibration->cameras[vpt.view].calibration->calibration_matrix,
            calibration->cameras[vpt.view].calibration->distortion_coefficients,
            proj,
            J_proj_cv);

        Eigen::Matrix<double, 2, 3> J_proj;
        cv::cv2eigen( J_proj_cv(cv::Range::all(), cv::Range(3, 6)), J_proj );

        Eigen::Matrix<double, 3, 9> A;
        A.setZero();
        A.block<1,3>(0,0) = mpt->position.transpose();
        A.block<1,3>(1,3) = mpt->position.transpose();
        A.block<1,3>(2,6) = mpt->position.transpose();

        Eigen::Matrix<double, 2, 3> J_wrt_position = - J_proj * world_to_camera[vpt.view].rotationMatrix();

        Eigen::Matrix<double, 2, 4> J_wrt_attitude = J_proj * rig_to_camera_R[vpt.view] * A * B * C;

        Eigen::Matrix<double, 2, 3> J_wrt_mappoint = J_proj * world_to_camera[vpt.view].rotationMatrix();

        h(2*i+0) = proj.front().x;
        h(2*i+1) = proj.front().y;

        for(int j=0; j<2; j++)
        {
            J.insert(2*i+j, 0) = J_wrt_position(j, 0);
            J.insert(2*i+j, 1) = J_wrt_position(j, 1);
            J.insert(2*i+j, 2) = J_wrt_position(j, 2);

            J.insert(2*i+j, 3) = J_wrt_attitude(j, 0);
            J.insert(2*i+j, 4) = J_wrt_attitude(j, 1);
            J.insert(2*i+j, 5) = J_wrt_attitude(j, 2);
            J.insert(2*i+j, 6) = J_wrt_attitude(j, 3);

            J.insert(2*i+j, 13+3*visible_points[i].local_index+0) = J_wrt_mappoint(j, 0);
            J.insert(2*i+j, 13+3*visible_points[i].local_index+1) = J_wrt_mappoint(j, 1);
            J.insert(2*i+j, 13+3*visible_points[i].local_index+2) = J_wrt_mappoint(j, 2);
        }
    }
}

void SLAMModuleEKF::jacobianOfQuaternionToRotationMatrix( const Eigen::Quaterniond& q, Eigen::Matrix<double, 9, 4>& J )
{
    const double qi = q.x();
    const double qj = q.y();
    const double qk = q.z();
    const double qr = q.w();

    /*
    Eigen::Matrix<double, 9, 1> ret;
    ret(0) = -2*(pow(qj, 2) + pow(qk, 2))/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) + 1;
    ret(1) = 2*(qi*qj - qk*qr)/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
    ret(2) = 2*(qi*qk + qj*qr)/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
    ret(3) = 2*(qi*qj + qk*qr)/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
    ret(4) = -2*(pow(qi, 2) + pow(qk, 2))/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) + 1;
    ret(5) = 2*(-qi*qr + qj*qk)/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
    ret(6) = 2*(qi*qk - qj*qr)/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
    ret(7) = 2*(qi*qr + qj*qk)/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
    ret(8) = -2*(pow(qi, 2) + pow(qj, 2))/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) + 1;
    */

    J( 0, 0 ) = -2*qi*(-2*pow(qj, 2) - 2*pow(qk, 2))/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
    J( 0, 1 ) = 4*qj*(pow(qj, 2) + pow(qk, 2))/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 4*qj/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
    J( 0, 2 ) = 4*qk*(pow(qj, 2) + pow(qk, 2))/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 4*qk/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
    J( 0, 3 ) = -2*qr*(-2*pow(qj, 2) - 2*pow(qk, 2))/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
    J( 1, 0 ) = -4*qi*(qi*qj - qk*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) + 2*qj/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
    J( 1, 1 ) = 2*qi/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) - 4*qj*(qi*qj - qk*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
    J( 1, 2 ) = -4*qk*(qi*qj - qk*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 2*qr/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
    J( 1, 3 ) = -2*qk/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) - 4*qr*(qi*qj - qk*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
    J( 2, 0 ) = -4*qi*(qi*qk + qj*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) + 2*qk/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
    J( 2, 1 ) = -4*qj*(qi*qk + qj*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) + 2*qr/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
    J( 2, 2 ) = 2*qi/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) - 4*qk*(qi*qk + qj*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
    J( 2, 3 ) = 2*qj/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) - 4*qr*(qi*qk + qj*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
    J( 3, 0 ) = -4*qi*(qi*qj + qk*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) + 2*qj/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
    J( 3, 1 ) = 2*qi/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) - 4*qj*(qi*qj + qk*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
    J( 3, 2 ) = -4*qk*(qi*qj + qk*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) + 2*qr/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
    J( 3, 3 ) = 2*qk/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) - 4*qr*(qi*qj + qk*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
    J( 4, 0 ) = 4*qi*(pow(qi, 2) + pow(qk, 2))/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 4*qi/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
    J( 4, 1 ) = -2*qj*(-2*pow(qi, 2) - 2*pow(qk, 2))/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
    J( 4, 2 ) = 4*qk*(pow(qi, 2) + pow(qk, 2))/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 4*qk/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
    J( 4, 3 ) = -2*qr*(-2*pow(qi, 2) - 2*pow(qk, 2))/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
    J( 5, 0 ) = -4*qi*(-qi*qr + qj*qk)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 2*qr/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
    J( 5, 1 ) = -4*qj*(-qi*qr + qj*qk)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) + 2*qk/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
    J( 5, 2 ) = 2*qj/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) - 4*qk*(-qi*qr + qj*qk)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
    J( 5, 3 ) = -2*qi/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) - 4*qr*(-qi*qr + qj*qk)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
    J( 6, 0 ) = -4*qi*(qi*qk - qj*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) + 2*qk/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
    J( 6, 1 ) = -4*qj*(qi*qk - qj*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 2*qr/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
    J( 6, 2 ) = 2*qi/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) - 4*qk*(qi*qk - qj*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
    J( 6, 3 ) = -2*qj/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) - 4*qr*(qi*qk - qj*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
    J( 7, 0 ) = -4*qi*(qi*qr + qj*qk)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) + 2*qr/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
    J( 7, 1 ) = -4*qj*(qi*qr + qj*qk)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) + 2*qk/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
    J( 7, 2 ) = 2*qj/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) - 4*qk*(qi*qr + qj*qk)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
    J( 7, 3 ) = 2*qi/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) - 4*qr*(qi*qr + qj*qk)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
    J( 8, 0 ) = 4*qi*(pow(qi, 2) + pow(qj, 2))/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 4*qi/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
    J( 8, 1 ) = 4*qj*(pow(qi, 2) + pow(qj, 2))/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 4*qj/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
    J( 8, 2 ) = -2*qk*(-2*pow(qi, 2) - 2*pow(qj, 2))/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
    J( 8, 3 ) = -2*qr*(-2*pow(qi, 2) - 2*pow(qj, 2))/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
}

