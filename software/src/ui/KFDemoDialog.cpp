#include <QToolBar>
#include <QVBoxLayout>
#include <QAction>
#include <QScrollArea>
#include <QSplitter>
#include "KFDemoDialog.h"
#include "KFDemoParametersDialog.h"

KFDemoDialog::KFDemoDialog(Project* proj, QWidget* parent) : QDialog(parent)
{
    mProject = proj;

    mEngine = new KFDEngine(this);
    mViewer = new KFDViewer(mEngine->posePort());
    mVideo = new VideoWidget();

    QToolBar* tb = new QToolBar();
    mActionStart = tb->addAction("Start");
    mActionStop = tb->addAction("Stop");

    mActionStop->setEnabled(false);

    QScrollArea* scroll = new QScrollArea();
    scroll->setAlignment(Qt::AlignCenter);
    scroll->setWidget(mVideo);

    QSplitter* splitter = new QSplitter();
    splitter->setOrientation(Qt::Horizontal);
    splitter->setChildrenCollapsible(false);
    splitter->addWidget(scroll);
    splitter->addWidget(mViewer);

    QVBoxLayout* l = new QVBoxLayout();
    l->addWidget(tb, 0);
    l->addWidget(splitter, 1);

    setLayout(l);
    setWindowTitle("KF Demo");

    connect(mActionStart, SIGNAL(triggered()), this, SLOT(onStartDemo()));
    connect(mActionStop, SIGNAL(triggered()), this, SLOT(onStopDemo()));
    connect(mEngine, SIGNAL(started()), this, SLOT(onEngineStarted()));
    connect(mEngine, SIGNAL(finished()), this, SLOT(onEngineStopped()));
}

void KFDemoDialog::onStartDemo()
{
    KFDemoParametersDialog* dlg = new KFDemoParametersDialog(mProject, mEngine, this);

    const int ret = dlg->exec();

    if(ret == QDialog::Accepted)
    {
        mActionStart->setEnabled(false);
        mActionStop->setEnabled(false);
        mEngine->start();
    }

    delete dlg;
}

void KFDemoDialog::onStopDemo()
{
    mEngine->requestInterruption();
    mActionStart->setEnabled(false);
    mActionStop->setEnabled(false);
}

void KFDemoDialog::onEngineStarted()
{
    mActionStart->setEnabled(false);
    mActionStop->setEnabled(true);
}

void KFDemoDialog::onEngineStopped()
{
    mActionStart->setEnabled(true);
    mActionStop->setEnabled(false);
}

