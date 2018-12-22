#pragma once

#include <QWidget>
#include <QTreeView>
#include "Project.h"

class Project;

class RigCalibrationPanel : public QWidget
{

public:

    RigCalibrationPanel(Project* project, QWidget* parent=nullptr);

protected:

    QTreeView* mView;
    QTextEdit* mText;
    Project* mProject;
};

