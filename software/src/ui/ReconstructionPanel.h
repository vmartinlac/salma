#pragma once

#include <QWidget>
#include <QTextEdit>
#include <QTreeView>

class Project;

class ReconstructionPanel : public QWidget
{

public:

    ReconstructionPanel(Project* project, QWidget* parent=nullptr);

protected:

    QTextEdit* mText;
    QTreeView* mView;
    Project* mProject;
};

