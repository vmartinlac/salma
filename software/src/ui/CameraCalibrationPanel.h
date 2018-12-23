#pragma once

#include <QTextEdit>
#include <QListView>
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

protected:

    QListView* mView;
    QTextEdit* mText;
    Project* mProject;
};

