#pragma once

#include <QWidget>
#include <QTextEdit>
#include <QListView>

class Project;

class RecordingPanel : public QWidget
{

public:

    RecordingPanel(Project* project, QWidget* parent=nullptr);

protected:

    Project* mProject;
    QListView* mView;
    QTextEdit* mText;
};

