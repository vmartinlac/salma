#pragma once

#include <QTextEdit>
#include <QListView>
#include <QWidget>

class Project;

class CameraCalibrationPanel : public QWidget
{
    Q_OBJECT

public:

    CameraCalibrationPanel(Project* project, QWidget* parent=nullptr);

protected slots:

    void onNew();
    void onRename();
    void onDelete();

    void onSelect(const QModelIndex&);

protected:

    QListView* mView;
    QTextEdit* mText;
    Project* mProject;
};

