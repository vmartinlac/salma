#include <QToolBar>
#include <QSplitter>
#include <QVBoxLayout>
#include "CameraCalibrationPanel.h"
#include "Project.h"

CameraCalibrationPanel::CameraCalibrationPanel(Project* project, QWidget* parent)
{
    mProject = project;
    mView = new QTreeView();
    mText = new QTextEdit();

    mView->setModel(mProject->cameraCalibrationModel());

    QToolBar* tb = new QToolBar();
    QAction* aNew = tb->addAction("New");
    QAction* aImport = tb->addAction("Import");
    QAction* aExport = tb->addAction("Export");
    QAction* aRename = tb->addAction("Rename");
    QAction* aDelete = tb->addAction("Delete");

    connect(aNew, SIGNAL(triggered()), this, SLOT(onNew()));
    connect(aImport, SIGNAL(triggered()), this, SLOT(onImport()));
    connect(aExport, SIGNAL(triggered()), this, SLOT(onExport()));
    connect(aRename, SIGNAL(triggered()), this, SLOT(onRename()));
    connect(aDelete, SIGNAL(triggered()), this, SLOT(onDelete()));

    QSplitter* splitter = new QSplitter();
    splitter->addWidget(mView);
    splitter->addWidget(mText);

    QVBoxLayout* lay = new QVBoxLayout();
    lay->addWidget(tb);
    lay->addWidget(splitter);

    setLayout(lay);
}

void CameraCalibrationPanel::onNew()
{
}

void CameraCalibrationPanel::onImport()
{
}

void CameraCalibrationPanel::onExport()
{
}

void CameraCalibrationPanel::onRename()
{
}

void CameraCalibrationPanel::onDelete()
{
}

