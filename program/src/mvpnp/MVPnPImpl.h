#pragma once

#include "MVPnP.h"

namespace MVPnP
{

    class SolverImpl : public Solver
    {
    public:

        SolverImpl();

        virtual ~SolverImpl();

        bool run( const std::vector<View>& views, Sophus::SE3d& rig_to_world, std::vector< std::vector<bool> >& inliers) override;

    protected:

        typedef Eigen::Matrix<double, 7, 1> TangentType;

        double computeError(TangentType& gradient);

        void applyIncrement(const TangentType& increment);

        Eigen::Vector3d quaternionToRodrigues(const Eigen::Quaterniond& q, Eigen::Matrix<double, 3, 4>& J);

    protected:

        Sophus::SE3d mWorldToRig;
        const std::vector<View>* mViews;
    };
}

