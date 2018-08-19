#pragma once

#include <QTabWidget>
#include <QLabel>
#include <QMainWindow>
#include "SLAMEngine.h"
#include "SLAMParameters.h"
#include "VideoWidget.h"
#include "StatsWidget.h"

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
    void ask_slam_parameters();
    void ask_video_input();

protected:

    QAction* m_a_start;
    QAction* m_a_stop;
    QAction* m_a_parameters;
    QAction* m_a_video;
    SLAMEngine* m_slam;
    ViewerWidget* m_viewer;
    VideoWidget* m_video;
    StatsWidget* m_stats;
    SLAMParameters m_slam_parameters;
    std::shared_ptr<Camera> m_camera;
};
