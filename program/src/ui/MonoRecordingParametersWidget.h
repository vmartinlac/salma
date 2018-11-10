#pragma once

#include <QCheckBox>
#include <QDialog>
#include <QSpinBox>
#include <QDir>
#include "CameraList.h"
#include "PathWidget.h"
#include "OperationParametersWidget.h"

class MonoRecordingParametersWidget : public OperationParametersWidget
{
    Q_OBJECT

public:

    MonoRecordingParametersWidget(QWidget* parent=nullptr);

    OperationPtr getOperation() override;

    QString name() override;

protected:

    CameraList* mCameraList;
    PathWidget* mPath;
    QCheckBox* mVisualizationOnly;
    QSpinBox* mMaxFrameRate;
};

