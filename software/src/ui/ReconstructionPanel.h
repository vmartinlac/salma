#pragma once

#include <QWidget>
#include <QTextEdit>
#include <QListView>

class Project;

class ReconstructionPanel : public QWidget
{
    Q_OBJECT

public:

    ReconstructionPanel(Project* project, QWidget* parent=nullptr);

protected slots:

    void onNew();
    void onShow();
    void onRename();
    void onDelete();

protected:

    QListView* mView;
    QTextEdit* mText;
    Project* mProject;
};

