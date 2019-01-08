
#pragma once

#include "SLAMDataStructures.h"
#include "SLAMModule.h"

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

    static void compute_f(
        const Eigen::VectorXd& X,
        double dt,
        Eigen::VectorXd& f,
        Eigen::SparseMatrix<double>& J);

protected:

    double mLastFrameTimestamp;
    std::vector<SLAMMapPointPtr> mLocalMap;
    Eigen::VectorXd mMu;
    Eigen::MatrixXd mSigma;
};

