#include <QToolBar>
#include <QMessageBox>
#include <QVBoxLayout>
#include <QScrollArea>
#include <QMessageBox>
#include <QIcon>
#include <QKeySequence>
#include <QSplitter>
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
        mActionStop->setShortcut(QKeySequence("Maj+F5"));
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

OperationDialog::~OperationDialog()
{
    if( mThread->isRunning() )
    {
        mThread->requestInterruption();
        mThread->wait();
        
        if(mOperation->success())
        {
            mOperation->discardResult();
        }
    }
}

void OperationDialog::startOperation()
{
    mThread->start();
    mActionStart->setEnabled(false);
    mActionStop->setEnabled(false);
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
    mActionStart->setEnabled(false);
    mActionStop->setEnabled(false);

    if( mOperation->success() )
    {
        QMessageBox::StandardButton ret = QMessageBox::question(this, "Result", "Save result?", QMessageBox::Yes|QMessageBox::No, QMessageBox::Yes);

        switch(ret)
        {
        case QMessageBox::Yes:
            mOperation->saveResult(mProject);
            break;
        case QMessageBox::No:
            mOperation->discardResult();
            break;
        default:
            throw std::runtime_error("internal error");
        }
    }
    else
    {
        QMessageBox::information(this, "Result", "Operation was not successful");
    }
}

