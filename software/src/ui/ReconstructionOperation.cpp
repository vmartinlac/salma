#include "ReconstructionOperation.h"

ReconstructionOperation::ReconstructionOperation()
{
}

ReconstructionOperation::~ReconstructionOperation()
{
}

const char* ReconstructionOperation::getName()
{
    return "Reconstruction";
}

bool ReconstructionOperation::before()
{
    return false;
}

bool ReconstructionOperation::step()
{
    return false;
}

void ReconstructionOperation::after()
{
}

bool ReconstructionOperation::success()
{
    return false;
}

bool ReconstructionOperation::saveResult(Project* project)
{
    return false;
}

void ReconstructionOperation::discardResult()
{
}

