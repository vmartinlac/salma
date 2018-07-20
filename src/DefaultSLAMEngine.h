#pragma once

#include <opencv2/core.hpp>
#include <Eigen/Eigen>
#include "SLAMEngine.h"

class DefaultSLAMEngine : public SLAMEngine
{
public:

    DefaultSLAMEngine();

    ~DefaultSLAMEngine() override;

    void initialize() override;

    void processNextView(Image* image) override;

    std::string name() override;

protected:

    struct Landmark
    {
        Eigen::Vector3d position;
        Eigen::Matrix3d sigma;
    };

    struct View
    {
        int num_key_points;
        Eigen::Array<float, Eigen::Dynamic, 2> key_points_positions;
    };

protected:

    std::vector<Landmark> m_map;
};

