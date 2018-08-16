#pragma once

#include <QThread>
#include "SLAMParameters.h"
#include "SLAMOutput.h"

class Camera;

class SLAMEngine : public QThread
{
public:

    static SLAMEngine* create(Camera* camera);

    SLAMEngine(QObject* parent=nullptr);

    virtual ~SLAMEngine();

    void setParameters(const SLAMParameters& params);

    SLAMOutput* getOutput();

protected:

    SLAMOutput* m_output;
    SLAMParameters m_parameters;
};

inline void SLAMEngine::setParameters(const SLAMParameters& params)
{
    m_parameters = params;
}

inline SLAMOutput* SLAMEngine::getOutput()
{
    return m_output;
}

