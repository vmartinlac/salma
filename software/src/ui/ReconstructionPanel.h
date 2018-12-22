#pragma once

#include <QWidget>
#include <QTextEdit>
#include <QListView>

class Project;

class ReconstructionPanel : public QWidget
{

public:

    ReconstructionPanel(Project* project, QWidget* parent=nullptr);

protected:

    QTextEdit* mText;
    QListView* mView;
    Project* mProject;
};

