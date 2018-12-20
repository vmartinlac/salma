#pragma once

#include <QMainWindow>

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    MainWindow(QWidget* parent=nullptr);

protected slots:

    void showProjectInformation();
    void showAvailableCameras();
    void clearProject();
    void about();
};
