#include <QToolBar>
#include <QScrollArea>
#include <QMessageBox>
#include <QAction>
#include <QIcon>
#include <QSplitter>
#include "ParametersDialog.h"
#include "MainWindow.h"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent)
{
    // set up the toolbar.

    {
        QToolBar* tb = addToolBar("ToolBar");

        tb->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);

        QAction* aConfigure = tb->addAction("Configure");
        QAction* aStart = tb->addAction("Start");
        QAction* aStop = tb->addAction("Stop");
        QAction* aAbout = tb->addAction("About");

        aConfigure->setIcon(QIcon::fromTheme("document-properties"));
        aStart->setIcon(QIcon::fromTheme("media-playback-start"));
        aStop->setIcon(QIcon::fromTheme("media-playback-stop"));
        aAbout->setIcon(QIcon::fromTheme("help-about"));

        QObject::connect(aConfigure, SIGNAL(triggered()), this, SLOT(configure()));
        QObject::connect(aStart, SIGNAL(triggered()), this, SLOT(startRecording()));
        QObject::connect(aStop, SIGNAL(triggered()), this, SLOT(stopRecording()));
        QObject::connect(aAbout, SIGNAL(triggered()), this, SLOT(about()));
    }

    // set up central widgets.

    {

        mVideoWidget = new VideoWidget();

        mInformationWidget = new InformationWidget();

        QScrollArea* scroll = new QScrollArea();
        scroll->setAlignment(Qt::AlignCenter);
        scroll->setWidget(mVideoWidget);

        QSplitter* splitter = new QSplitter();
        splitter->setChildrenCollapsible(false);
        splitter->setOrientation(Qt::Vertical);
        splitter->addWidget(scroll);
        splitter->addWidget(mInformationWidget);

        setCentralWidget(splitter);
    }

    setWindowTitle("Video Recorder");
}

void MainWindow::configure()
{
    ParametersDialog* dlg = new ParametersDialog(this);

    dlg->exec();

    delete dlg;
}

void MainWindow::startRecording()
{
}

void MainWindow::stopRecording()
{
}

void MainWindow::about()
{
    QMessageBox::information(this, "About", "Tool to record a video. Written by Victor Martin Lac 2018.");
}

