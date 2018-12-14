#pragma once

#include <QMainWindow>
#include "SLAMDataStructures.h"
#include "ViewerWidget.h"
#include "VisualizationData.h"
#include "VisualizationSettings.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    MainWindow(QWidget* parent=nullptr);

    ~MainWindow();

protected:

    VisualizationDataPort* mVisualizationData;
    VisualizationSettingsPort* mVisualizationSettings;

protected slots:

    void openReconstruction();
    void visualizationSettings();
    void exportPointCloud();
    void about();
};

