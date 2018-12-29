#pragma once

#include <QWidget>
#include <QListView>
#include "Project.h"

class Project;

class RigCalibrationPanel : public QWidget
{
    Q_OBJECT
public:

    RigCalibrationPanel(Project* project, QWidget* parent=nullptr);

protected slots:

    void onNew();
    void onRename();
    void onDelete();

protected:

    QListView* mView;
    QTextEdit* mText;
    Project* mProject;
};

