#pragma once

#include <random>
#include "MVPnPLM.h"

namespace MVPnP
{
    class SolverRANSACLM : public SolverLM
    {
    public:

        SolverRANSACLM();

        virtual ~SolverRANSACLM();

        void setInlierRate(double rate);
        void setInlierThreshold(double threshold);
        void setSuccessProbability(double probability);

        int getNumberOfRANSACIterations();

        bool run( const std::vector<View>& views, Sophus::SE3d& rig_to_world, bool use_ransac, std::vector< std::vector<bool> >& inliers) override;

    private:

        struct Sample
        {
            Sample(int v, int p)
            {
                view = v;
                point = p;
            }

            int view;
            int point;
        };

    private:

        void extractSample(
            const std::vector<View>& views,
            int maxsize,
            std::vector<View>& subviews);

        void buildWholeSampleArray(
            const std::vector<View>& views);

    private:

        std::default_random_engine mEngine;
        std::vector<Sample> mWholeSample;
        double mInlierThreshold;
        double mInlierRate;
        double mSuccessProbability;
        int mMinimumNumberOfObservations;
    };
}

