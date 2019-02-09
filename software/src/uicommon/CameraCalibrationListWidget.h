#pragma once

#include <QComboBox>

class Project;

class CameraCalibrationListWidget : public QComboBox
{
    Q_OBJECT
public:
    
    CameraCalibrationListWidget(Project* project, QWidget* parent=nullptr);
    ~CameraCalibrationListWidget();

    int getCameraCalibrationId();

public slots:

    void refreshList();

protected:

    Project* mProject;
};

