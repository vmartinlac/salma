#pragma once

#include <QDialog>
#include <QDir>
#include <QLineEdit>
#include <QComboBox>
#include "Camera.h"
#include "OperationParametersWidget.h"

class MonoRecordingParametersWidget : public OperationParametersWidget
{
public:

    MonoRecordingParametersWidget(QWidget* parent=nullptr);

    OperationPtr getOperation() override;

    QString name() override;
};

/*
#include "RecordingParameters.h"

class RecordingParametersDialog : public QDialog
{
    Q_OBJECT
public:

    RecordingParametersDialog(RecordingParameters* parameters, QWidget* parent=nullptr);

public slots:

    int exec() override;

protected slots:

    void accept();
    void selectOutputDirectory();

protected:

    QLineEdit* mPath;
    QComboBox* mCameraList;

    RecordingParameters* mParameters;
};
*/

