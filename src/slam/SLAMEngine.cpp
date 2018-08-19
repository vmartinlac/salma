#include "SLAMEngine.h"

SLAMEngine::SLAMEngine()
{
    m_output = new SLAMOutput(this);
}

SLAMEngine::~SLAMEngine()
{
    ;
}

void SLAMEngine::setCamera(std::shared_ptr<Camera> camera)
{
    m_camera = std::move(camera);
}

void SLAMEngine::setParameters(const SLAMParameters& params)
{
    if( isRunning() )
    {
        throw std::runtime_error("internal error");
    }
    else
    {
        m_parameters = params;
    }
}

SLAMOutput* SLAMEngine::getOutput()
{
    return m_output;
}

