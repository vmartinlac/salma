#include <QListWidget>
#include <QInputDialog>
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

    connect(project, SIGNAL(reconstructionModelChanged()), this, SLOT(onModelChanged()));

    mView->setModel(mProject->reconstructionModel());

    connect(mView, SIGNAL(clicked(const QModelIndex&)), this, SLOT(onSelect(const QModelIndex&)));
    connect(mView, SIGNAL(doubleClicked(const QModelIndex&)), this, SLOT(onShow()));

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
    }
}

void ReconstructionPanel::onShow()
{
    int reconstruction_id = -1;
    bool ok = true;
    SLAMReconstructionPtr reconstruction;

    if(ok)
    {
        reconstruction_id = mProject->reconstructionModel()->indexToId(mView->currentIndex());
        ok = (reconstruction_id >= 0);
    }

    if(ok)
    {
        ok = mProject->loadReconstruction(reconstruction_id, reconstruction) && bool(reconstruction);

        if(ok == false)
        {
            QMessageBox::critical(this, "Error", "Could not load reconstruction!");
        }
    }

    if(ok)
    {
        std::cout << "OK" << std::endl;
        std::cout << reconstruction->frames.size() << std::endl;
        std::cout << reconstruction->name << std::endl;
        std::cout << reconstruction->frames.back()->views[0].projections.size() << std::endl;
        // TODO
        /*
        RecordingPlayerDialog* dlg = new RecordingPlayerDialog(header, this);
        dlg->exec();
        delete dlg;
        */
    }
}

void ReconstructionPanel::onRename()
{
    QString text;
    int reconstruction_id = -1;
    bool ok = true;

    if(ok)
    {
        reconstruction_id = mProject->reconstructionModel()->indexToId(mView->currentIndex());
        ok = (reconstruction_id >= 0);
    }

    if(ok)
    {
        bool accepted;
        text = QInputDialog::getText(this, "Rename", "New name?", QLineEdit::Normal, "", &accepted);

        ok = accepted && (text.isEmpty() == false);

        if(accepted == true && ok == false)
        {
            QMessageBox::critical(this, "Error", "Incorrect name!");
        }
    }

    if(ok)
    {
        ok = mProject->renameReconstruction(reconstruction_id, text);

        if(ok == false)
        {
            QMessageBox::critical(this, "Error", "Could not rename camera calibration!");
        }
    }
}

void ReconstructionPanel::onDelete()
{
    QMessageBox::critical(this, "Error", "Not implemented");
}

void ReconstructionPanel::onSelect(const QModelIndex& ind)
{
    QString text;
    int reconstruction_id;
    bool ok = true;

    if(ok)
    {
        reconstruction_id = mProject->reconstructionModel()->indexToId(ind);
        ok = (reconstruction_id >= 0);
    }

    if(ok)
    {
        ok = mProject->describeReconstruction(reconstruction_id, text);
    }

    if(ok)
    {
        mText->setText(text);
    }
    else
    {
        mText->setText(QString());
    }
}

void ReconstructionPanel::onModelChanged()
{
    mText->clear();
}

