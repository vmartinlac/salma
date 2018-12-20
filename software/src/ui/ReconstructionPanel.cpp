#include <QListWidget>
#include <QTextEdit>
#include <QToolBar>
#include <QSplitter>
#include <QVBoxLayout>
#include "ReconstructionPanel.h"

ReconstructionPanel::ReconstructionPanel(QWidget* parent)
{
    QToolBar* tb = new QToolBar();
    tb->addAction("New");
    tb->addAction("New from");
    tb->addAction("Inspect");
    tb->addAction("Rename");
    tb->addAction("Delete");

    QSplitter* splitter = new QSplitter();
    splitter->addWidget(new QListWidget());
    splitter->addWidget(new QTextEdit());

    QVBoxLayout* lay = new QVBoxLayout();
    lay->addWidget(tb);
    lay->addWidget(splitter);

    setLayout(lay);
}

