
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
    void operator()() override;

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

    Eigen::Vector4d quaternionProduct(const Eigen::Vector4d& P, const Eigen::Vector4d& Q, Eigen::Matrix<double, 4, 8>& J);

    Eigen::Vector4d rotationVectorToQuaternion(const Eigen::Vector3d& v, Eigen::Matrix<double, 4, 3>& J);

protected:

    double mLastFrameTimestamp;
    std::vector<SLAMMapPointPtr> mLocalMap;
    Eigen::VectorXd mMu;
    Eigen::MatrixXd mSigma;
    std::default_random_engine mEngine;
};

