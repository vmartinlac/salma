#pragma once

#include <QMainWindow>
#include "Project.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    MainWindow(QWidget* parent=nullptr);

protected slots:

    void newProject();
    void openProject();
    void closeProject();
    void showProjectInformation();
    void showAvailableCameras();
    void clearProject();
    void about();

protected:

    Project* mProject;
};
