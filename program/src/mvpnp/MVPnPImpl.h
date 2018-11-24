#pragma once

#include <lbfgs.h>
#include "MVPnP.h"

namespace MVPnP
{

    class SolverImpl : public Solver
    {
    public:

        SolverImpl();

        virtual ~SolverImpl();

        bool run( const std::vector<View>& views, Sophus::SE3d& rig_to_world, std::vector< std::vector<bool> >& inliers) override;

        const std::vector<View>* getViews();

    protected:

        static lbfgsfloatval_t evaluateProc(
            void* data,
            const lbfgsfloatval_t* x,
            lbfgsfloatval_t* gradient,
            const int n,
            const lbfgsfloatval_t step);

        static int progressProc(
            void* data,
            const lbfgsfloatval_t* x,
            const lbfgsfloatval_t* gradient,
            const lbfgsfloatval_t fx,
            const lbfgsfloatval_t xnorm,
            const lbfgsfloatval_t gnorm,
            const lbfgsfloatval_t step,
            int n,
            int k,
            int ls);

    protected:

        const std::vector<View>* mViews;
    };
}

