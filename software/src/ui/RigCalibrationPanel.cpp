#include <QListWidget>
#include <QTextEdit>
#include <QToolBar>
#include <QSplitter>
#include <QVBoxLayout>
#include "RigCalibrationPanel.h"

RigCalibrationPanel::RigCalibrationPanel(Project* project, QWidget* parent)
{
    mProject = project;
    mView = new QTreeView();
    mText = new QTextEdit();

    mView->setModel(mProject->rigCalibrationModel());

    QToolBar* tb = new QToolBar();
    tb->addAction("New");
    tb->addAction("Import");
    tb->addAction("Export");
    tb->addAction("Rename");
    tb->addAction("Delete");

    QSplitter* splitter = new QSplitter();
    splitter->addWidget(mView);
    splitter->addWidget(mText);

    QVBoxLayout* lay = new QVBoxLayout();
    lay->addWidget(tb);
    lay->addWidget(splitter);

    setLayout(lay);
}

