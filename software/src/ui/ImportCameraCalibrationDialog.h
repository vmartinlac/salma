#pragma once

#include <QDialog>
#include <QLineEdit>
#include "PathWidget.h"

class Project;

class ImportCameraCalibrationDialog : public QDialog
{
    Q_OBJECT

public:

    ImportCameraCalibrationDialog(Project* project, QWidget* parent=nullptr);

protected:

    void accept() override;

protected:

    PathWidget* mPath;
    QLineEdit* mName;
    Project* mProject;
};

