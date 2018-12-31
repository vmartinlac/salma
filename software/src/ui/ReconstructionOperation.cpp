#include <QMessageBox>
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

bool ReconstructionOperation::uibefore(QWidget* parent, Project* project)
{
    return false;
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

void ReconstructionOperation::uiafter(QWidget* parent, Project* project)
{
}

