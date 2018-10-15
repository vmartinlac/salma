#pragma once

#include <QThread>
#include <memory>
#include "SLAMParameters.h"
#include "Camera.h"
#include "SLAMOutput.h"

class SLAMEngine : public QThread
{
public:

    static SLAMEngine* create();

    SLAMEngine();

    virtual ~SLAMEngine();

    void setParameters(const SLAMParameters& params);

    void setCamera(std::shared_ptr<Camera> camera);

    SLAMOutput* getOutput();

protected:

    SLAMParameters m_parameters;
    std::shared_ptr<Camera> m_camera;
    SLAMOutput* m_output;
};

