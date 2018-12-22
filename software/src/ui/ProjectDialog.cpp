#include <QPushButton>
#include <QMessageBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFormLayout>
#include "Project.h"
#include "ProjectDialog.h"

ProjectDialog::ProjectDialog(Project* project, QWidget* parent) : QDialog(parent)
{
    mProject = project;
    mPath = new PathWidget(PathWidget::GET_EXISTING_DIRECTORY);

    QFormLayout* form = new QFormLayout();
    form->addRow("Path", mPath);

    QPushButton* btnok = new QPushButton("OK");
    QPushButton* btncancel = new QPushButton("Cancel");

    QHBoxLayout* hlay = new QHBoxLayout();
    hlay->addWidget(btnok);
    hlay->addWidget(btncancel);

    QVBoxLayout* vlay = new QVBoxLayout();
    vlay->addLayout(form);
    vlay->addLayout(hlay);

    setLayout(vlay);
    setWindowTitle("Open project");

    connect(btnok, SIGNAL(clicked()), this, SLOT(accept()));
    connect(btncancel, SIGNAL(clicked()), this, SLOT(reject()));
}

void ProjectDialog::accept()
{
    const QString path = mPath->path();

    if( mProject->open(path) )
    {
        QDialog::accept();
    }
    else
    {
        QMessageBox::critical(this, "Error", "Could not load project!");
    }
}

