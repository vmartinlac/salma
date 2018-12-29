#pragma once

#include <QComboBox>

class Project;

class RigCalibrationListWidget : public QComboBox
{
    Q_OBJECT
public:
    
    RigCalibrationListWidget(Project* project, QWidget* parent=nullptr);
    ~RigCalibrationListWidget();

    int getRigCalibrationId();

public slots:

    void refreshList();

protected:

    Project* mProject;
};

