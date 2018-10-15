#pragma once

#include <QDialog>
#include <QSpinBox>
#include <QDir>
#include <QLineEdit>
#include <QComboBox>
#include "Camera.h"
#include "OperationParametersWidget.h"

class StereoRecordingParametersWidget : public OperationParametersWidget
{
    Q_OBJECT

public:

    StereoRecordingParametersWidget(QWidget* parent=nullptr);

    OperationPtr getOperation() override;

    QString name() override;

protected slots:

    void selectOutputDirectory();

protected:

    QLineEdit* mPath;
    QComboBox* mCameraList;
    QSpinBox* mMaxFrameRate;
};

