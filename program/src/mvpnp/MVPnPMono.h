#pragma once

#include "MVPnP.h"

namespace MVPnP
{

    class SolverMono : public Solver
    {
    public:

        SolverMono();

        virtual ~SolverMono();

        bool run( const std::vector<View>& views, Sophus::SE3d& rig_to_world, bool use_ransac, std::vector< std::vector<bool> >& inliers) override;
    };
}

