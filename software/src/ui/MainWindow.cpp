#include <QMenu>
#include <QMessageBox>
#include <QFileDialog>
#include <QApplication>
#include <QStatusBar>
#include <QListWidget>
#include <QToolBar>
#include <QSplitter>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTextEdit>
#include <QTabWidget>
#include <QMenuBar>
#include <QAction>
#include "MainWindow.h"
#include "CameraCalibrationPanel.h"
#include "RigCalibrationPanel.h"
#include "RecordingPanel.h"
#include "ReconstructionPanel.h"
#include "AboutDialog.h"
#include "ProjectDialog.h"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent)
{
    mProject = new Project(this);

    QMenu* menuProject = menuBar()->addMenu("Project");
    QAction* aOpen = menuProject->addAction("Open");
    QAction* aInformation = menuProject->addAction("Information");
    QAction* aCameras = menuProject->addAction("Available cameras");
    QAction* aClear = menuProject->addAction("Clear");
    QAction* aClose = menuProject->addAction("Close");
    QAction* aQuit = menuProject->addAction("Quit");

    QMenu* menuHelp = menuBar()->addMenu("Help");
    QAction* aAbout = menuHelp->addAction("About");

    connect(aOpen, SIGNAL(triggered()), this, SLOT(openProject()));
    connect(aClose, SIGNAL(triggered()), this, SLOT(closeProject()));
    connect(aQuit, SIGNAL(triggered()), QApplication::instance(), SLOT(quit()));
    connect(aClear, SIGNAL(triggered()), this, SLOT(clearProject()));
    connect(aCameras, SIGNAL(triggered()), this, SLOT(showAvailableCameras()));
    connect(aInformation, SIGNAL(triggered()), this, SLOT(showProjectInformation()));
    connect(aAbout, SIGNAL(triggered()), this, SLOT(about()));

    QTabWidget* tab = new QTabWidget();
    tab->addTab(new CameraCalibrationPanel(mProject), "Camera Calibration");
    tab->addTab(new RigCalibrationPanel(mProject), "Rig Calibration");
    tab->addTab(new RecordingPanel(mProject), "Recording");
    tab->addTab(new ReconstructionPanel(mProject), "Reconstruction");

    statusBar()->showMessage("SALMA v1.0");

    setCentralWidget(tab);
    setWindowTitle("Salma");
}

void MainWindow::about()
{
    AboutDialog* dlg = new AboutDialog(this);
    dlg->exec();
    delete dlg;
}

void MainWindow::showAvailableCameras()
{
    QMessageBox::critical(this, "Error", "Not implemented");
}

void MainWindow::showProjectInformation()
{
    QMessageBox::critical(this, "Error", "Not implemented");
}


void MainWindow::clearProject()
{
    QMessageBox::critical(this, "Error", "Not implemented");
}

void MainWindow::openProject()
{
    /*
    ProjectDialog* dlg = new ProjectDialog(mProject, this);
    dlg->exec();
    delete dlg;
    */
    QString ret = QFileDialog::getExistingDirectory(this, "Open project");

    if( ret.isEmpty() == false )
    {
        const bool ok = mProject->open(ret);

        if(ok == false)
        {
            QMessageBox::critical(this, "Error", "Could not load project!");
        }
    }
}

void MainWindow::closeProject()
{
    mProject->close();
}

