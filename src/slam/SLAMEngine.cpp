#include "SLAMEngine.h"

SLAMEngine::SLAMEngine(QObject* parent) : QThread(parent)
{
    m_output = new SLAMOutput(this);
}

SLAMEngine::~SLAMEngine()
{
    ;
}

