#pragma once

#include <QMainWindow>
#include "SLAMDataStructures.h"
#include "ViewerWidget.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    MainWindow(QWidget* parent=nullptr);

    ~MainWindow();

protected:

    std::shared_ptr<FrameList> mFrames;

protected slots:

    void openReconstruction();
    void visualizationSettings();
    void exportPointCloud();
    void about();
};

