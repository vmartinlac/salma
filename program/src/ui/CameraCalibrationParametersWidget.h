
#pragma once

#include "CameraList.h"
#include "PathWidget.h"
#include "OperationParametersWidget.h"

class CameraCalibrationParametersWidget : public OperationParametersWidget
{
    Q_OBJECT

public:

    CameraCalibrationParametersWidget(QWidget* parent=nullptr);

    OperationPtr getOperation() override;

    QString name() override;

protected:

    PathWidget* mPath;
    CameraList* mCameraList;
};

