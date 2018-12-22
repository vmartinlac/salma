#pragma once

#include <QWidget>
#include <QListView>
#include "Project.h"

class Project;

class RigCalibrationPanel : public QWidget
{

public:

    RigCalibrationPanel(Project* project, QWidget* parent=nullptr);

protected:

    QListView* mView;
    QTextEdit* mText;
    Project* mProject;
};

