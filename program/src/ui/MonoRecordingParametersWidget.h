#pragma once

#include <QDialog>
#include <QDir>
#include <QLineEdit>
#include <QComboBox>
#include "Camera.h"
#include "OperationParametersWidget.h"

class MonoRecordingParametersWidget : public OperationParametersWidget
{
    Q_OBJECT

public:

    MonoRecordingParametersWidget(QWidget* parent=nullptr);

    OperationPtr getOperation() override;

    QString name() override;

protected slots:

    void selectOutputDirectory();

protected:

    QLineEdit* mPath;
    QComboBox* mCameraList;
};

