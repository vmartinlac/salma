#include <QToolBar>
#include <QScrollArea>
#include <QMessageBox>
#include <QIcon>
#include <QKeySequence>
#include <QSplitter>
#include "AboutDialog.h"
#include "CameraCalibrationParametersDialog.h"
#include "CameraCalibrationMainWindow.h"

CameraCalibrationMainWindow::CameraCalibrationMainWindow(QWidget* parent) : QMainWindow(parent)
{
    // set up the toolbar.

    {
        QToolBar* tb = addToolBar("ToolBar");

        tb->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);

        mActionConfigure = tb->addAction("Configure");
        mActionStart = tb->addAction("Start");
        mActionStop = tb->addAction("Stop");
        mActionAbout = tb->addAction("About");

        mActionConfigure->setIcon(QIcon::fromTheme("document-properties"));
        mActionStart->setIcon(QIcon::fromTheme("media-playback-start"));
        mActionStop->setIcon(QIcon::fromTheme("media-playback-stop"));
        mActionAbout->setIcon(QIcon::fromTheme("help-about"));

        mActionConfigure->setShortcut(QKeySequence("Alt+C"));

        QObject::connect(mActionConfigure, SIGNAL(triggered()), this, SLOT(configure()));
        QObject::connect(mActionStart, SIGNAL(triggered()), this, SLOT(startCameraCalibration()));
        QObject::connect(mActionStop, SIGNAL(triggered()), this, SLOT(stopCameraCalibration()));
        QObject::connect(mActionAbout, SIGNAL(triggered()), this, SLOT(about()));

        mActionStop->setEnabled(false);
    }

    // set up central widgets.

    {
        mVideoWidget = new VideoWidget();

        mStatsWidget = new CameraCalibrationStatsWidget();

        QScrollArea* scroll = new QScrollArea();
        scroll->setAlignment(Qt::AlignCenter);
        scroll->setWidget(mVideoWidget);

        QSplitter* splitter = new QSplitter();
        splitter->setChildrenCollapsible(false);
        splitter->setOrientation(Qt::Vertical);
        splitter->addWidget(scroll);
        splitter->addWidget(mStatsWidget);

        setCentralWidget(splitter);
    }

    // set up engine and other data structures.

    {
        mParameters = new CameraCalibrationParameters(this);
        mEngine = new CameraCalibrationThread(mParameters, mVideoWidget->getPort(), mStatsWidget->getPort(), this);

        QObject::connect(mEngine, SIGNAL(started()), this, SLOT(engineStarted()));
        QObject::connect(mEngine, SIGNAL(finished()), this, SLOT(engineStopped()));
    }

    setWindowTitle("Camera Calibration");
}

void CameraCalibrationMainWindow::configure()
{
    CameraCalibrationParametersDialog* dlg = new CameraCalibrationParametersDialog(mParameters, this);

    int ret = dlg->exec();

    if(ret == QDialog::Accepted)
    {
        ;
    }

    delete dlg;
}

void CameraCalibrationMainWindow::startCameraCalibration()
{
    mEngine->start();
    mActionStart->setEnabled(false);
    mActionStop->setEnabled(false);
}

void CameraCalibrationMainWindow::stopCameraCalibration()
{
    mEngine->requestInterruption();
    mActionStart->setEnabled(false);
    mActionStop->setEnabled(false);
}

void CameraCalibrationMainWindow::about()
{
    AboutDialog* dlg = new AboutDialog(this);
    dlg->exec();
    delete dlg;
}

void CameraCalibrationMainWindow::engineStarted()
{
    mActionStart->setEnabled(false);
    mActionStop->setEnabled(true);
}

void CameraCalibrationMainWindow::engineStopped()
{
    mActionStart->setEnabled(true);
    mActionStop->setEnabled(false);
}

