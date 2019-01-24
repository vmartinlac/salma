
#pragma once

#include "SLAMDataStructures.h"
#include "SLAMModule.h"
#include <random>

class SLAMModuleEKF : public SLAMModule
{
public:

    SLAMModuleEKF(SLAMContextPtr con);
    ~SLAMModuleEKF() override;

    bool init() override;
    SLAMModuleResult operator()() override;

protected:

    struct VisiblePoint
    {
        int local_index;
        int view;
        int keypoint;
    };

protected:

    void initializeState();
    void prepareLocalMap();
    void ekfPrediction();
    void ekfUpdate();
    void exportResult();

    void compute_f(
        const Eigen::VectorXd& X,
        double dt,
        Eigen::VectorXd& f,
        Eigen::SparseMatrix<double>& J);

    void compute_h(
        const Eigen::VectorXd& X,
        const std::vector<VisiblePoint>& visible_points,
        Eigen::VectorXd& h,
        Eigen::SparseMatrix<double>& J);

    Eigen::Vector4d rotationVectorToQuaternion(
        const Eigen::Vector3d& v,
        Eigen::Matrix<double, 4, 3>& J);

    void jacobianOfQuaternionToRotationMatrix(
        const Eigen::Quaterniond& q,
        Eigen::Matrix<double, 9, 4>& J);

protected:

    int mLastFrameId;
    double mLastFrameTimestamp;
    SLAMFramePtr mCurrentFrame;
    std::vector<SLAMMapPointPtr> mLocalMap;
    Eigen::VectorXd mMu;
    Eigen::MatrixXd mSigma;
    std::default_random_engine mEngine;
};

