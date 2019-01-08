#include "KFDViewer.h"

KFDViewer::KFDViewer(KFDPosePort* pose, QWidget* parent) : ViewerWidgetBase(parent)
{
    mPose = pose;
}

