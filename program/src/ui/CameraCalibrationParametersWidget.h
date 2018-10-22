
#pragma once

#include <QComboBox>
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
    QComboBox* mCameraList;
};

