
#pragma once

#include <QDialog>
#include <QDir>
#include <QLineEdit>
#include <QComboBox>
#include "Camera.h"
#include "OperationParametersWidget.h"

class CameraCalibrationParametersWidget : public OperationParametersWidget
{
public:

    CameraCalibrationParametersWidget(QWidget* parent=nullptr);

    OperationPtr getOperation() override;

    QString name() override;
};

/*
#include "CameraCalibrationParameters.h"

class CameraCalibrationParametersDialog : public QDialog
{
    Q_OBJECT

public:

    CameraCalibrationParametersDialog(CameraCalibrationParameters* parameters, QWidget* parent=nullptr);

public slots:

    int exec() override;

protected slots:

    void accept();
    void selectOutputPath();

protected:

    QLineEdit* mPath;
    QComboBox* mCameraList;

    CameraCalibrationParameters* mParameters;
};
*/

