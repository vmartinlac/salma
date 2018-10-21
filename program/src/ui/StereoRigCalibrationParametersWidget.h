
#pragma once

#include <QLineEdit>
#include <QComboBox>
#include "OperationParametersWidget.h"

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

    QComboBox* mLeftCamera;
    QComboBox* mRightCamera;
    QLineEdit* mOutputPath;
};

