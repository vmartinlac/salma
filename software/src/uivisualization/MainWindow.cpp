#include <QIcon>
#include <QApplication>
#include <QMessageBox>
#include <QMenuBar>
#include <QAction>
#include <QTabWidget>
#include <QToolBar>
#include "OpenReconstructionDialog.h"
#include "ExportPointCloudDialog.h"
#include "AboutDialog.h"
#include "MainWindow.h"
#include "InspectorWidget.h"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent)
{
    mVisualizationData = new VisualizationDataPort(this);
    mVisualizationSettings = new VisualizationSettingsPort(this);

    ViewerWidget* viewer = new ViewerWidget(mVisualizationData, mVisualizationSettings);
    //InspectorWidget* inspector = new InspectorWidget();

    QMenu* menuFile = menuBar()->addMenu("File");
    QMenu* menuView = menuBar()->addMenu("View");
    QMenu* menuHelp = menuBar()->addMenu("Help");

    QAction* a_open_reconstruction = menuFile->addAction("Open reconstruction");
    QAction* a_export_point_cloud = menuFile->addAction("Export point cloud");
    QAction* a_quit = menuFile->addAction("Quit");
    QAction* a_about = menuHelp->addAction("About");

    QAction* a_home = menuView->addAction("Home");
    QAction* a_trajectory = menuView->addAction("Show trajectory");
    QAction* a_rig = menuView->addAction("Show rig");
    QAction* a_mappoints = menuView->addAction("Show mappoints");
    QAction* a_densepoints = menuView->addAction("Show densepoints");
    a_trajectory->setCheckable(true);
    a_rig->setCheckable(true);
    a_mappoints->setCheckable(true);
    a_densepoints->setCheckable(true);
    a_trajectory->setChecked(true);
    a_rig->setChecked(true);
    a_mappoints->setChecked(true);
    a_densepoints->setChecked(true);

    connect(a_open_reconstruction, SIGNAL(triggered()), this, SLOT(openReconstruction()));
    connect(a_export_point_cloud, SIGNAL(triggered()), this, SLOT(exportPointCloud()));
    connect(a_quit, SIGNAL(triggered()), QApplication::instance(), SLOT(quit()));
    connect(a_about, SIGNAL(triggered()), this, SLOT(about()));

    connect(a_home, SIGNAL(triggered()), viewer, SLOT(home()));
    connect(a_trajectory, SIGNAL(toggled(bool)), this, SLOT(showTrajectory(bool)));
    connect(a_rig, SIGNAL(toggled(bool)), this, SLOT(showRig(bool)));
    connect(a_mappoints, SIGNAL(toggled(bool)), this, SLOT(showMapPoints(bool)));
    connect(a_densepoints, SIGNAL(toggled(bool)), this, SLOT(showDensePoints(bool)));

    //QTabWidget* tab_widget = new QTabWidget();
    //tab_widget->addTab(viewer, "Viewer");
    //tab_widget->addTab(inspector, "Inspector");

    setCentralWidget(viewer);
    setWindowTitle("Salma Visualization");
    resize(800, 600);
}

MainWindow::~MainWindow()
{
}

void MainWindow::openReconstruction()
{
    OpenReconstructionDialog* dlg = new OpenReconstructionDialog(mVisualizationData, this);
    const int ret = dlg->exec();

    /*
    if( ret == QDialog::Accepted )
    {
        mFrames = dlg->getReconstruction();
    }
    */

    delete dlg;
}

void MainWindow::exportPointCloud()
{
    /*
    ExportPointCloudDialog* dlg = new ExportPointCloudDialog(this);
    dlg->exec();
    delete dlg;
    */
    QMessageBox::critical(this, "Error", "Not implemented!");
}

void MainWindow::about()
{
    AboutDialog* dlg = new AboutDialog(this);
    dlg->exec();
    delete dlg;
}

void MainWindow::showRig(bool value)
{
    mVisualizationSettings->beginWrite();
    mVisualizationSettings->data().show_rig = value;
    mVisualizationSettings->endWrite();
}

void MainWindow::showTrajectory(bool value)
{
    mVisualizationSettings->beginWrite();
    mVisualizationSettings->data().show_trajectory = value;
    mVisualizationSettings->endWrite();
}

void MainWindow::showMapPoints(bool value)
{
    mVisualizationSettings->beginWrite();
    mVisualizationSettings->data().show_mappoints = value;
    mVisualizationSettings->endWrite();
}

void MainWindow::showDensePoints(bool value)
{
    mVisualizationSettings->beginWrite();
    mVisualizationSettings->data().show_densepoints = value;
    mVisualizationSettings->endWrite();
}

