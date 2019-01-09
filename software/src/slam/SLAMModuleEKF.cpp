#include <set>
#include <fstream>
#include <iostream>
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
    mMu.setZero();
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
        //const double symm_err = ( new_sigma - new_sigma.transpose() ).norm();
        //std::cout << symm_err << std::endl;
    }

    mLocalMap.swap(new_local_map);
    mMu.swap(new_mu);
    mSigma.swap(new_sigma);

    //
    std::ofstream f("tmp"+std::to_string(frame->id)+".csv");
    for(int i=0; i<mSigma.rows(); i++)
    {
        for(int j=0; j<mSigma.cols(); j++)
        {
            f << mSigma(i,j) << ' ';
        }
        f << std::endl;
    }
    f.close();
    //
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
    //const double symm_err = ( new_sigma - new_sigma.transpose() ).norm();
    //std::cout << "symm = " << symm_err << std::endl;

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

    const Eigen::Vector3d r = X.segment<3>(0);
    const Eigen::Vector4d q = X.segment<4>(3);
    const Eigen::Vector3d v = X.segment<3>(7);
    const Eigen::Vector3d w = X.segment<3>(10);

    Eigen::Matrix<double, 4, 3> Js;
    const Eigen::Vector4d s = rotationVectorToQuaternion(w*dt, Js);

    Eigen::Matrix<double, 4, 8> Jt;
    Eigen::Vector4d t = quaternionProduct(q, s, Jt);

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

Eigen::Vector4d SLAMModuleEKF::quaternionProduct(const Eigen::Vector4d& P, const Eigen::Vector4d& Q, Eigen::Matrix<double, 4, 8>& J)
{
    const double pi = P(0);
    const double pj = P(1);
    const double pk = P(2);
    const double pr = P(3);

    const double qi = Q(0);
    const double qj = Q(1);
    const double qk = Q(2);
    const double qr = Q(3);

    const double ui = pr * qi + qr * pi + (pj*qk - pk*qj);
    const double uj = pr * qj + qr * pj + (pk*qi - pi*qk);
    const double uk = pr * qk + qr * pk + (pi*qj - pj*qi);
    const double ur = pr*qr - pi*qi - pj*qj - pk*qk;

    Eigen::Vector4d ret;

    ret(0) = ui;
    ret(1) = uj;
    ret(2) = uk;
    ret(3) = ur;

    J( 0, 0 ) = qk;
    J( 0, 1 ) = qj;
    J( 0, 2 ) = -qi;
    J( 0, 3 ) = qr;
    J( 0, 4 ) = pk;
    J( 0, 5 ) = -pj;
    J( 0, 6 ) = pi;
    J( 0, 7 ) = pr;
    J( 1, 0 ) = -qj;
    J( 1, 1 ) = qk;
    J( 1, 2 ) = qr;
    J( 1, 3 ) = qi;
    J( 1, 4 ) = pj;
    J( 1, 5 ) = pk;
    J( 1, 6 ) = -pr;
    J( 1, 7 ) = pi;
    J( 2, 0 ) = qi;
    J( 2, 1 ) = -qr;
    J( 2, 2 ) = qk;
    J( 2, 3 ) = qj;
    J( 2, 4 ) = -pi;
    J( 2, 5 ) = pr;
    J( 2, 6 ) = pk;
    J( 2, 7 ) = pj;
    J( 3, 0 ) = -qr;
    J( 3, 1 ) = -qi;
    J( 3, 2 ) = -qj;
    J( 3, 3 ) = qk;
    J( 3, 4 ) = -pr;
    J( 3, 5 ) = -pi;
    J( 3, 6 ) = -pj;
    J( 3, 7 ) = pk;

    return ret;
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

