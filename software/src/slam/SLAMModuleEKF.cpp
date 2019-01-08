#include <set>
#include <map>
#include "SLAMModuleEKF.h"

SLAMModuleEKF::SLAMModuleEKF(SLAMContextPtr con) : SLAMModule(con)
{
}

SLAMModuleEKF::~SLAMModuleEKF()
{
}

bool SLAMModuleEKF::init()
{
    SLAMContextPtr con = context();

    //const double scale_factor = con->configuration->features_scale_factor;

    return true;
}

void SLAMModuleEKF::operator()()
{
    std::cout << "   EXTENDED KALMAN FILTER" << std::endl;

    SLAMReconstructionPtr reconstr = context()->reconstruction;

    if( reconstr->frames.empty() ) throw std::runtime_error("internal error");

    SLAMFramePtr frame = reconstr->frames.back();

    if(frame->aligned_wrt_previous_frame)
    {
        prepareLocalMap();
        ekfPrediction();
        ekfUpdate();
        exportResult();
    }
    else
    {
        initializeState();
    }

    mLastFrameTimestamp = frame->timestamp;
}

void SLAMModuleEKF::initializeState()
{
    if( context()->reconstruction->frames.empty() ) throw std::runtime_error("internal error");

    SLAMFramePtr frame = context()->reconstruction->frames.back();

    mLocalMap.clear();

    mMu.resize(13);
    mMu(0) = frame->frame_to_world.translation().x();
    mMu(1) = frame->frame_to_world.translation().y();
    mMu(2) = frame->frame_to_world.translation().z();
    mMu(3) = frame->frame_to_world.unit_quaternion().x();
    mMu(4) = frame->frame_to_world.unit_quaternion().y();
    mMu(5) = frame->frame_to_world.unit_quaternion().z();
    mMu(6) = frame->frame_to_world.unit_quaternion().w();
    mMu(7) = 0.0;
    mMu(8) = 0.0;
    mMu(9) = 0.0;
    mMu(10) = 0.0;
    mMu(11) = 0.0;
    mMu(12) = 0.0;

    const double position_sdev = 0.1;
    const double attitude_sdev = 0.1;
    const double linear_momentum_sdev = 20.0;
    const double angular_momentum_sdev = M_PI*0.4;

    mSigma.resize(13, 13);
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

    SLAMFramePtr frame;

    // retrieve last frame.

    if( context()->reconstruction->frames.empty() ) throw std::runtime_error("internal error");

    frame = context()->reconstruction->frames.back();

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
            for(SLAMTrack& t : frame->views[i].tracks)
            {
                if( t.mappoint && new_local_map_content.count(t.mappoint->id) == 0 )
                {
                    new_local_map.push_back(t.mappoint);
                    new_local_map_content.insert(t.mappoint->id);
                }
            }
        }
    }

    // if there are too many points in local map, remove some.

    const int max_local_map_size = 340; // TODO: put this in config file.

    if( new_local_map.size() > max_local_map_size )
    {
        auto comp = [] (SLAMMapPointPtr P1, SLAMMapPointPtr P2)
        {
            return (P1->last_seen_frame_id > P2->last_seen_frame_id);
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
                new_mu.segment<3>(13 + i*3) = new_local_map[i]->position;
                //new_sigma.block<3,3>(13 + i*3, 13 + i*3) = new_local_map[i]->position_covariance;
                new_sigma.block<3,3>(13 + i*3, 13 + i*3) = Eigen::Matrix3d::Identity() * (3.0*3.0); // TODO !

                // TODO: covariance between camera pose and mappoint position.
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
        const double symm_err = ( new_sigma - new_sigma.transpose() ).norm();
        std::cout << symm_err << std::endl;
    }

    mLocalMap.swap(new_local_map);
    mMu.swap(new_mu);
    mSigma.swap(new_sigma);
}

void SLAMModuleEKF::ekfPrediction()
{
    Eigen::SparseMatrix<double> Q;
    Eigen::SparseMatrix<double> J;

    Eigen::VectorXd new_mu;
    Eigen::MatrixXd new_sigma;

    if( context()->reconstruction->frames.empty() ) throw std::runtime_error("internal error");

    SLAMFramePtr frame = context()->reconstruction->frames.back();

    const double dt = frame->timestamp - mLastFrameTimestamp;

    const int num_landmarks = mLocalMap.size();

    const int dim = 13 + 3*num_landmarks;

    // set Q.
    {
        // TODO: get these constants from configuration.
        const double sigma_v = dt*5.3; //dt*0.1;
        const double sigma_w = dt*1.2;

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

    mMu.swap(new_mu);
    mSigma.swap(new_sigma);
}

void SLAMModuleEKF::ekfUpdate()
{
}

void SLAMModuleEKF::exportResult()
{
    // export frame pose.

    {
        SLAMFramePtr frame;

        if( context()->reconstruction->frames.empty() ) throw std::runtime_error("internal error");

        frame = context()->reconstruction->frames.back();

        Eigen::Vector3d position;
        Eigen::Quaterniond attitude;

        position = mMu.segment<3>(0);
        attitude.x() = mMu(3);
        attitude.y() = mMu(4);
        attitude.z() = mMu(5);
        attitude.w() = mMu(6);

        frame->frame_to_world.translation() = position;
        frame->frame_to_world.setQuaternion(attitude);
        frame->pose_covariance = mSigma.block<13,13>(0,0);
    }

    // export mappoints positions.
    
    {
        for(int i=0; i<mLocalMap.size(); i++)
        {
            mLocalMap[i]->position = mMu.segment<3>(13+3*i);
            mLocalMap[i]->position_covariance = mSigma.block<3,3>(13+3*i, 13+3*i);
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

    const double x1 = X(0);
    const double x2 = X(1);
    const double x3 = X(2);

    const double a1 = X(3);
    const double a2 = X(4);
    const double a3 = X(5);
    const double a0 = X(6);

    const double v1 = X(7);
    const double v2 = X(8);
    const double v3 = X(9);

    double w1 = X(10);
    double w2 = X(11);
    double w3 = X(12);

    // TODO: handle small rotations in a better way.

    double norm_w = std::sqrt(w1*w1 + w2*w2 + w3*w3);
    double axis1;
    double axis2;
    double axis3;
    const double theta = 0.5*norm_w*dt;
    const double cos_theta = std::cos(theta);
    const double sin_theta = std::sin(theta);
    double sin_theta_over_norm_w;

    if( norm_w < 1.0e-10 )
    {
        norm_w = 0.0;
        axis1 = 1.0;
        axis2 = 0.0;
        axis3 = 0.0;
        sin_theta_over_norm_w = 0.0;
    }
    else
    {
        axis1 = w1/norm_w;
        axis2 = w2/norm_w;
        axis3 = w3/norm_w;
        sin_theta_over_norm_w = sin_theta / norm_w;
    }

    const double r1 = sin_theta * axis1;
    const double r2 = sin_theta * axis2;
    const double r3 = sin_theta * axis3;
    const double r0 = cos_theta;

    // fill f.

    f.resize(dim);

    f.head<13>() <<
        x1 + dt*v1,
        x2 + dt*v2,
        x3 + dt*v3,
        a0*r1 + r0*a1 + (a2*r3 - a3*r2),
        a0*r2 + r0*a2 + (a3*r1 - a1*r3),
        a0*r3 + r0*a3 + (a1*r2 - a2*r1),
        a0*r0 - a1*r1 - a2*r2 - a3*r3,
        v1,
        v2,
        v3,
        w1,
        w2,
        w3;

    //f.segment<4>(3).normalize();

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

    // Generated automatically by python script system2.py.
    // BEGIN
    Jrm.insert(3,3) = r0;
    Jrm.insert(3,4) = -r1;
    Jrm.insert(3,5) = -r2;
    Jrm.insert(3,6) = -r3;
    Jrm.insert(3,10) = -0.5*a0*dt*r1 - 0.5*a1*dt*axis1*axis1*cos_theta + 1.0*a1*axis1*axis1*sin_theta_over_norm_w - a1*norm_w*sin_theta - 0.5*a2*dt*axis1*axis2*cos_theta + 1.0*a2*axis1*axis2*sin_theta_over_norm_w - 0.5*a3*dt*axis1*axis3*cos_theta + 1.0*a3*axis1*axis3*sin_theta_over_norm_w;
    Jrm.insert(3,11) = -0.5*a0*dt*r2 - 0.5*a1*dt*axis1*axis2*cos_theta + 1.0*a1*axis1*axis2*sin_theta_over_norm_w - 0.5*a2*dt*axis2*axis2*cos_theta + 1.0*a2*axis2*axis2*sin_theta_over_norm_w - a2*norm_w*sin_theta - 0.5*a3*dt*axis2*axis3*cos_theta + 1.0*a3*axis2*axis3*sin_theta_over_norm_w;
    Jrm.insert(3,12) = -0.5*a0*dt*r3 - 0.5*a1*dt*axis1*axis3*cos_theta + 1.0*a1*axis1*axis3*sin_theta_over_norm_w - 0.5*a2*dt*axis2*axis3*cos_theta + 1.0*a2*axis2*axis3*sin_theta_over_norm_w - 0.5*a3*dt*axis3*axis3*cos_theta + 1.0*a3*axis3*axis3*sin_theta_over_norm_w - a3*norm_w*sin_theta;
    Jrm.insert(4,3) = r1;
    Jrm.insert(4,4) = r0;
    Jrm.insert(4,5) = r3;
    Jrm.insert(4,6) = -r2;
    Jrm.insert(4,10) = 0.5*a0*dt*axis1*axis1*cos_theta - 1.0*a0*axis1*axis1*sin_theta_over_norm_w + a0*norm_w*sin_theta - 0.5*a1*dt*r1 + 0.5*a2*dt*axis1*axis3*cos_theta - 1.0*a2*axis1*axis3*sin_theta_over_norm_w - 0.5*a3*dt*axis1*axis2*cos_theta + 1.0*a3*axis1*axis2*sin_theta_over_norm_w;
    Jrm.insert(4,11) = 0.5*a0*dt*axis1*axis2*cos_theta - 1.0*a0*axis1*axis2*sin_theta_over_norm_w - 0.5*a1*dt*r2 + 0.5*a2*dt*axis2*axis3*cos_theta - 1.0*a2*axis2*axis3*sin_theta_over_norm_w - 0.5*a3*dt*axis2*axis2*cos_theta + 1.0*a3*axis2*axis2*sin_theta_over_norm_w - a3*norm_w*sin_theta;
    Jrm.insert(4,12) = 0.5*a0*dt*axis1*axis3*cos_theta - 1.0*a0*axis1*axis3*sin_theta_over_norm_w - 0.5*a1*dt*r3 + 0.5*a2*dt*axis3*axis3*cos_theta - 1.0*a2*axis3*axis3*sin_theta_over_norm_w + a2*norm_w*sin_theta - 0.5*a3*dt*axis2*axis3*cos_theta + 1.0*a3*axis2*axis3*sin_theta_over_norm_w;
    Jrm.insert(5,3) = r2;
    Jrm.insert(5,4) = -r3;
    Jrm.insert(5,5) = r0;
    Jrm.insert(5,6) = r1;
    Jrm.insert(5,10) = 0.5*a0*dt*axis1*axis2*cos_theta - 1.0*a0*axis1*axis2*sin_theta_over_norm_w - 0.5*a1*dt*axis1*axis3*cos_theta + 1.0*a1*axis1*axis3*sin_theta_over_norm_w - 0.5*a2*dt*r1 + 0.5*a3*dt*axis1*axis1*cos_theta - 1.0*a3*axis1*axis1*sin_theta_over_norm_w + a3*norm_w*sin_theta;
    Jrm.insert(5,11) = 0.5*a0*dt*axis2*axis2*cos_theta - 1.0*a0*axis2*axis2*sin_theta_over_norm_w + a0*norm_w*sin_theta - 0.5*a1*dt*axis2*axis3*cos_theta + 1.0*a1*axis2*axis3*sin_theta_over_norm_w - 0.5*a2*dt*r2 + 0.5*a3*dt*axis1*axis2*cos_theta - 1.0*a3*axis1*axis2*sin_theta_over_norm_w;
    Jrm.insert(5,12) = 0.5*a0*dt*axis2*axis3*cos_theta - 1.0*a0*axis2*axis3*sin_theta_over_norm_w - 0.5*a1*dt*axis3*axis3*cos_theta + 1.0*a1*axis3*axis3*sin_theta_over_norm_w - a1*norm_w*sin_theta - 0.5*a2*dt*r3 + 0.5*a3*dt*axis1*axis3*cos_theta - 1.0*a3*axis1*axis3*sin_theta_over_norm_w;
    Jrm.insert(6,3) = r3;
    Jrm.insert(6,4) = r2;
    Jrm.insert(6,5) = -r1;
    Jrm.insert(6,6) = r0;
    Jrm.insert(6,10) = 0.5*a0*dt*axis1*axis3*cos_theta - 1.0*a0*axis1*axis3*sin_theta_over_norm_w + 0.5*a1*dt*axis1*axis2*cos_theta - 1.0*a1*axis1*axis2*sin_theta_over_norm_w - 0.5*a2*dt*axis1*axis1*cos_theta + 1.0*a2*axis1*axis1*sin_theta_over_norm_w - a2*norm_w*sin_theta - 0.5*a3*dt*r1;
    Jrm.insert(6,11) = 0.5*a0*dt*axis2*axis3*cos_theta - 1.0*a0*axis2*axis3*sin_theta_over_norm_w + 0.5*a1*dt*axis2*axis2*cos_theta - 1.0*a1*axis2*axis2*sin_theta_over_norm_w + a1*norm_w*sin_theta - 0.5*a2*dt*axis1*axis2*cos_theta + 1.0*a2*axis1*axis2*sin_theta_over_norm_w - 0.5*a3*dt*r2;
    Jrm.insert(6,12) = 0.5*a0*dt*axis3*axis3*cos_theta - 1.0*a0*axis3*axis3*sin_theta_over_norm_w + a0*norm_w*sin_theta + 0.5*a1*dt*axis2*axis3*cos_theta - 1.0*a1*axis2*axis3*sin_theta_over_norm_w - 0.5*a2*dt*axis1*axis3*cos_theta + 1.0*a2*axis1*axis3*sin_theta_over_norm_w - 0.5*a3*dt*r3;
    // END

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

