#include "SLAMOutput.h"

SLAMOutput::SLAMOutput(QObject* parent) : QObject(parent)
{
    clear();
}

void SLAMOutput::clear()
{
    mode = QString();
    image.release();
    position.setZero();
    attitude.setIdentity();
    linear_velocity.setZero();
    angular_velocity.setZero();
}

