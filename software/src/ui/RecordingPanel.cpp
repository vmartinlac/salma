#include <QListWidget>
#include <QTextEdit>
#include <QToolBar>
#include <QSplitter>
#include <QVBoxLayout>
#include "RecordingPanel.h"
#include "Project.h"

RecordingPanel::RecordingPanel(Project* project, QWidget* parent)
{
    mProject = project;
    mText = new QTextEdit();
    mView = new QListView();

    mView->setModel(mProject->recordingModel());

    QToolBar* tb = new QToolBar();
    tb->addAction("New");
    tb->addAction("Play");
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

