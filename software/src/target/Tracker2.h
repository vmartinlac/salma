#pragma once

#include "TrackerBase.h"

namespace target
{
    class Tracker2 : public TrackerBase
    {
    public:

        Tracker2();

        bool track( const cv::Mat& image, bool absolute_pose=true ) override;

        void clear() override;

    protected:
    };
}

