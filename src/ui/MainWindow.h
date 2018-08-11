#pragma once

#include <QLabel>
#include <QMainWindow>
#include "SLAMEngine.h"

class ViewerWidget;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    MainWindow(
        SLAMEngine* slam,
        QWidget* parent=nullptr);

protected slots:

    void about();
    void start_slam();
    void stop_slam();
    void slam_started();
    void slam_stopped();

protected:

    QAction* m_a_start;
    QAction* m_a_stop;
    SLAMEngine* m_slam;
    ViewerWidget* m_viewer;
};
