#pragma once

#include "ViewerWidgetBase.h"
#include "KFDPose.h"

class KFDViewer : public ViewerWidgetBase
{
    Q_OBJECT

public:

    KFDViewer(KFDPosePort* pose, QWidget* parent=nullptr);

protected:

    KFDPosePort* mPose;
};

