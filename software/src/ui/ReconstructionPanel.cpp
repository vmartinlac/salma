#include <QListWidget>
#include <QMessageBox>
#include <QTextEdit>
#include <QToolBar>
#include <QSplitter>
#include <QVBoxLayout>
#include "ReconstructionPanel.h"
#include "Project.h"
#include "OperationDialog.h"
#include "NewReconstructionDialog.h"

ReconstructionPanel::ReconstructionPanel(Project* project, QWidget* parent)
{
    mProject = project;
    mText = new QTextEdit();
    mView = new QListView();

    mView->setModel(mProject->reconstructionModel());

    mText->setReadOnly(true);
    
    QToolBar* tb = new QToolBar();
    QAction* aNew = tb->addAction("New");
    QAction* aShow = tb->addAction("Show");
    QAction* aRename = tb->addAction("Rename");
    QAction* aDelete = tb->addAction("Delete");

    connect(aNew, SIGNAL(triggered()), this, SLOT(onNew()));
    connect(aShow, SIGNAL(triggered()), this, SLOT(onShow()));
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

void ReconstructionPanel::onNew()
{
    OperationPtr op;

    NewReconstructionDialog* dlg = new NewReconstructionDialog(mProject, this);
    dlg->exec();
    op = dlg->getOperation();
    delete dlg;

    if(op)
    {
        OperationDialog* opdlg = new OperationDialog(mProject, op, this);
        opdlg->exec();
        delete opdlg;

        mProject->reconstructionModel()->refresh();
    }
}

void ReconstructionPanel::onShow()
{
    QMessageBox::critical(this, "Error", "Not implemented");
}

void ReconstructionPanel::onRename()
{
    QMessageBox::critical(this, "Error", "Not implemented");
}

void ReconstructionPanel::onDelete()
{
    QMessageBox::critical(this, "Error", "Not implemented");
}

