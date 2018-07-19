#pragma once

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

protected:

    std::vector<Landmark> m_map;
};

