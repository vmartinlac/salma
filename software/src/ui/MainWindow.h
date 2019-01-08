#pragma once

#include <QMainWindow>
#include "Project.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    MainWindow(QWidget* parent=nullptr);

public slots:

    void loadProjectGivenOnCommandLine(const QString& path);

protected slots:

    void newProject();
    void openProject();
    void closeProject();
    void showAvailableCameras();
    void clearProject();
    void kfdemo();
    void about();

protected:

    Project* mProject;
};
