
#pragma once

#include <QLineEdit>
#include <QComboBox>
#include "OperationParametersWidget.h"

class CameraCalibrationParametersWidget : public OperationParametersWidget
{
    Q_OBJECT

public:

    CameraCalibrationParametersWidget(QWidget* parent=nullptr);

    OperationPtr getOperation() override;

    QString name() override;

protected slots:

    void selectOutputPath();

protected:

    QLineEdit* mPath;
    QComboBox* mCameraList;
};

