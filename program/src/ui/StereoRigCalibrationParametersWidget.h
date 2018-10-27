
#pragma once

#include <QLineEdit>
#include "CameraList.h"
#include "OperationParametersWidget.h"
#include "PathWidget.h"

class StereoRigCalibrationParametersWidget : public OperationParametersWidget
{
    Q_OBJECT

public:

    StereoRigCalibrationParametersWidget(QWidget* parent=nullptr);

    OperationPtr getOperation() override;

    QString name() override;

protected:

    PathWidget* mPathToLeftCalibrationData;
    PathWidget* mPathToRightCalibrationData;
    CameraList* mLeftCamera;
    CameraList* mRightCamera;
    PathWidget* mOutputPath;
};

