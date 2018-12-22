#pragma once

#include <QWidget>
#include <QTextEdit>
#include <QTreeView>

class Project;

class RecordingPanel : public QWidget
{

public:

    RecordingPanel(Project* project, QWidget* parent=nullptr);

protected:

    Project* mProject;
    QTreeView* mView;
    QTextEdit* mText;
};

