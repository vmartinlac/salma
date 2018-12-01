#pragma once

#include "MVPnP.h"

namespace MVPnP
{

    class SolverImpl : public Solver
    {
    public:

        SolverImpl();

        virtual ~SolverImpl();

        bool run( const std::vector<View>& views, Sophus::SE3d& rig_to_world, bool use_ransac, std::vector< std::vector<bool> >& inliers) override;

    protected:

        typedef Eigen::Matrix<double, 7, 1> IncrementType;

        typedef Eigen::Matrix<double, Eigen::Dynamic, 7> JacobianType;

    protected:

        bool runLM(
            const std::vector<View>& views,
            Sophus::SE3d& rig_to_world,
            std::vector< std::vector<bool> >& inliers);

        double computeErrorResidualsAndJacobianOfF(
            const Sophus::SE3d& world_to_rig,
            Eigen::Matrix<double, Eigen::Dynamic, 7>& JF,
            Eigen::VectorXd& residuals );

        void applyIncrement(
            Sophus::SE3d& world_to_rig,
            const IncrementType& increment);

        void printError(
            double error);

    protected:

        const std::vector<View>* mViews;
        int mTotalNumberOfPoints;
        int mMaxNumberOfIterations;
        bool mPrintError;
        double mLMFactor;
        double mLMFirstLambda;
    };
}

