#pragma once

#include <QDialog>
#include "PathWidget.h"

class Project;

class ProjectDialog : public QDialog
{
public:

    ProjectDialog(Project* project, QWidget* parent=nullptr);

protected:

    void accept() override;

protected:

    Project* mProject;
    PathWidget* mPath;
};
