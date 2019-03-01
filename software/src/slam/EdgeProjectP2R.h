#pragma once

#include <g2o/types/sba/types_sba.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include "StereoRigCalibration.h"

class EdgeProjectP2R : public g2o::BaseBinaryEdge<2, cv::Point2f, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>
{
public:

    EdgeProjectP2R();

    void setView(int view);

    void setRigCalibration(StereoRigCalibrationPtr rig);

    void computeError() override;

    bool read(std::istream& is) override;

    bool write(std::ostream& os) const override;

    void linearizeOplus() override;

protected:

    StereoRigCalibrationPtr mRigCalibration;
    int mView;
};

