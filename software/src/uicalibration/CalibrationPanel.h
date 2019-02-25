#pragma once

#include <QWidget>
#include <QListView>
#include <QTextEdit>
#include "Project.h"

class Project;

class CalibrationPanel : public QWidget
{
    Q_OBJECT
public:

    CalibrationPanel(Project* project, QWidget* parent=nullptr);

protected slots:

    void onNew();
    void onRename();
    void onDelete();
    void onSelect(const QModelIndex& ind);
    void onModelChanged();

protected:

    QListView* mView;
    QTextEdit* mText;
    Project* mProject;
};

