#pragma once

#include <string>
#include <Eigen/Eigen>

class Image;

class SLAMEngine
{

public:

    typedef Eigen::Array<float, Eigen::Dynamic, 3, Eigen::RowMajor> MapArray;

    typedef Eigen::Array<float, Eigen::Dynamic, 6, Eigen::RowMajor> CloudArray;

public:

    virtual ~SLAMEngine();

    static SLAMEngine* createDefaultSLAMEngine();

    virtual std::string name() = 0;

    virtual void initialize() = 0;

    virtual void processNextView(Image* image) = 0;

    MapArray& map();

    CloudArray& cloud();

protected:

    MapArray m_map_array;
    CloudArray m_cloud_array;
};

inline SLAMEngine::MapArray& SLAMEngine::map()
{
    return m_map_array;
}

inline SLAMEngine::CloudArray& SLAMEngine::cloud()
{
    return m_cloud_array;
}

