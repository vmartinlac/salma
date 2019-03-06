#pragma once

#include <memory>
#include "ElasIntf.h"
#include "elas.h"

class ElasIntfImpl : public ElasIntf
{
public:

    ElasIntfImpl();

    void compute(
        cv::InputArray left,
        cv::InputArray right,
        cv::OutputArray disparity) override;

    int getMinDisparity() const override;
    void setMinDisparity(int minDisparity) override;

    int getNumDisparities() const override;
    void setNumDisparities(int numDisparities) override;

    int getBlockSize() const override;
    void setBlockSize(int blockSize) override;

    int getSpeckleWindowSize() const override;
    void setSpeckleWindowSize(int speckleWindowSize) override;

    int getSpeckleRange() const override;
    void setSpeckleRange(int speckleRange) override;

    int getDisp12MaxDiff() const override;
    void setDisp12MaxDiff(int disp12MaxDiff) override;

protected:

    int mNumDisparities;
    std::unique_ptr<Elas> mElas;
};

