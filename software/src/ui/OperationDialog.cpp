#include <QToolBar>
#include <QMessageBox>
#include <QVBoxLayout>
#include <QScrollArea>
#include <QMessageBox>
#include <QIcon>
#include <QKeySequence>
#include <QSplitter>
#include <iostream>
#include "OperationDialog.h"

OperationDialog::OperationDialog(Project* proj, OperationPtr op, QWidget* parent) : QDialog(parent)
{
    mProject = proj;
    mOperation = op;

    // set up the toolbar.

    QToolBar* tb = nullptr;
    QSplitter* splitter = nullptr;

    {
        tb = new QToolBar("ToolBar");

        tb->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);

        mActionStart = tb->addAction("Start");
        mActionStop = tb->addAction("Stop");

        mActionStart->setIcon(QIcon::fromTheme("media-playback-start"));
        mActionStop->setIcon(QIcon::fromTheme("media-playback-stop"));

        QObject::connect(mActionStart, SIGNAL(triggered()), this, SLOT(startOperation()));
        QObject::connect(mActionStop, SIGNAL(triggered()), this, SLOT(stopOperation()));

        mActionStop->setEnabled(false);

        mActionStart->setShortcut(QKeySequence("F5"));
        mActionStop->setShortcut(QKeySequence("F6"));
    }

    // set up central widgets.

    {
        mVideoWidget = new VideoWidget();

        mStatsWidget = new StatsWidget();

        QScrollArea* scroll = new QScrollArea();
        scroll->setAlignment(Qt::AlignCenter);
        scroll->setWidget(mVideoWidget);

        splitter = new QSplitter();
        splitter->setChildrenCollapsible(false);
        splitter->setOrientation(Qt::Vertical);
        splitter->addWidget(scroll);
        splitter->addWidget(mStatsWidget);
    }

    // set up engine and other data structures.

    {
        mThread = new OperationThread( mVideoWidget->getPort(), mStatsWidget->getPort(), this );
        mThread->setOperation(mOperation);

        QObject::connect(mThread, SIGNAL(started()), this, SLOT(operationStarted()));
        QObject::connect(mThread, SIGNAL(finished()), this, SLOT(operationStopped()));
    }

    QVBoxLayout* lay = new QVBoxLayout();
    lay->addWidget(tb);
    lay->addWidget(splitter);

    setLayout(lay);
    setWindowTitle(mOperation->getName());
}

void OperationDialog::closeEvent(QCloseEvent* ev)
{
    if(mThread->isRunning())
    {
        //QMessageBox::critical(this, "Error", "Please stop operation before leaving!");
        stopOperation();
        ev->ignore();
    }
    else
    {
        QDialog::closeEvent(ev);
    }
}

OperationDialog::~OperationDialog()
{
    // this should not happen! 
    if( mThread->isRunning() )
    {
        std::cerr << "Dialog must not be destroyed while operation thread is running!" << std::endl;
        abort();
    }
}

void OperationDialog::startOperation()
{
    mActionStart->setEnabled(false);
    mActionStop->setEnabled(false);

    const bool ok = mOperation->uibefore(this, mProject);

    if(ok)
    {
        mThread->start();
    }
    else
    {
        mActionStart->setEnabled(true);
        mActionStop->setEnabled(false);
    }
}

void OperationDialog::stopOperation()
{
    mThread->requestInterruption();
    mActionStart->setEnabled(false);
    mActionStop->setEnabled(false);
}

void OperationDialog::operationStarted()
{
    mActionStart->setEnabled(false);
    mActionStop->setEnabled(true);
}

void OperationDialog::operationStopped()
{
    mOperation->uiafter(this, mProject);
    mActionStart->setEnabled(false);
    mActionStop->setEnabled(false);
}

