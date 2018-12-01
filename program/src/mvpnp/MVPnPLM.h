#pragma once

#include "MVPnP.h"

namespace MVPnP
{

    class SolverLM : public Solver
    {
    public:

        SolverLM();

        virtual ~SolverLM();

        bool run(
            const std::vector<View>& views,
            Sophus::SE3d& rig_to_world,
            bool use_ransac,
            std::vector< std::vector<bool> >& inliers) override;

    private:

        typedef Eigen::Matrix<double, 7, 1> IncrementType;

        typedef Eigen::Matrix<double, Eigen::Dynamic, 7> JacobianType;

        struct Parameters
        {
            int maxNumberOfIterations;
            bool printError;
            double LMFactor;
            double LMFirstLambda;
        };

        struct State
        {
            const std::vector<View>* views;
            int totalNumberOfPoints;
        };

    private:

        double computeErrorResidualsAndJacobianOfF(
            const Sophus::SE3d& world_to_rig,
            Eigen::Matrix<double, Eigen::Dynamic, 7>& JF,
            Eigen::VectorXd& residuals );

        void applyIncrement(
            Sophus::SE3d& world_to_rig,
            const IncrementType& increment );

        void printError(
            double error );

        static void fillSelection(
            const std::vector<View>& views,
            std::vector< std::vector<bool> >& selection,
            bool value );

    private:

        State mState;
        Parameters mParameters;
    };
}

