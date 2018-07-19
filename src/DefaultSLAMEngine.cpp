#include "DefaultSLAMEngine.h"

SLAMEngine* SLAMEngine::createDefaultSLAMEngine()
{
    return new DefaultSLAMEngine();
}

void DefaultSLAMEngine::initialize()
{
    ;
}

void DefaultSLAMEngine::processNextView(Image* image)
{
    ;
}

DefaultSLAMEngine::DefaultSLAMEngine()
{
}

DefaultSLAMEngine::~DefaultSLAMEngine()
{
    ;
}

std::string DefaultSLAMEngine::name()
{
    return "SLAM engine";
}
