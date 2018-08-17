#include "SLAMEngine.h"

SLAMEngine::SLAMEngine(QObject* parent) : QThread(parent)
{
    m_output = new SLAMOutput(this);
}

SLAMEngine::~SLAMEngine()
{
    ;
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

