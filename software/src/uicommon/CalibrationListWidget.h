#pragma once

#include <QComboBox>

class Project;

class CalibrationListWidget : public QComboBox
{
    Q_OBJECT
public:
    
    CalibrationListWidget(Project* project, QWidget* parent=nullptr);
    ~CalibrationListWidget();

    int getCalibrationId();

public slots:

    void refreshList();

protected:

    Project* mProject;
};

