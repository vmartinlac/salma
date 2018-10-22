
#pragma once

#include <QLineEdit>
#include <QComboBox>
#include "OperationParametersWidget.h"
#include "PathWidget.h"

class StereoRigCalibrationParametersWidget : public OperationParametersWidget
{
    Q_OBJECT

public:

    StereoRigCalibrationParametersWidget(QWidget* parent=nullptr);

    OperationPtr getOperation() override;

    QString name() override;

protected slots:

    void selectOutputPath();

protected:

    PathWidget* mPathToLeftCalibrationData;
    PathWidget* mPathToRightCalibrationData;
    QComboBox* mLeftCamera;
    QComboBox* mRightCamera;
    QLineEdit* mOutputPath;
};

