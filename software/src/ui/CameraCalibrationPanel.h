#pragma once

#include <QTextEdit>
#include <QTreeView>
#include <QWidget>
#include "CameraCalibrationModel.h"

class Project;

class CameraCalibrationPanel : public QWidget
{
    Q_OBJECT

public:

    CameraCalibrationPanel(Project* project, QWidget* parent=nullptr);

protected slots:

    void onNew();
    void onRename();
    void onImport();
    void onExport();
    void onDelete();

protected:

    QTreeView* mView;
    QTextEdit* mText;
    Project* mProject;
};

