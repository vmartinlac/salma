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

    mLastFrameTimestamp = frame->timestamp;
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

