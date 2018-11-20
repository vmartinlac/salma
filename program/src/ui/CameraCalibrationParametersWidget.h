
#pragma once

#include <QSpinBox>
#include "CameraList.h"
#include "PathWidget.h"
#include "OperationParametersWidget.h"
#include "TargetParametersWidget.h"

class CameraCalibrationParametersWidget : public OperationParametersWidget
{
    Q_OBJECT

public:

    CameraCalibrationParametersWidget(QWidget* parent=nullptr);

    OperationPtr getOperation() override;

    QString name() override;

protected:

    PathWidget* mPath;
    TargetParametersWidget* mTargetParameters;
    CameraList* mCameraList;
    QSpinBox* mNumViews;
};

