#include <QStackedLayout>
#include <QFormLayout>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include "OpenReconstructionDialog.h"

OpenReconstructionDialog::OpenReconstructionDialog(QWidget* p) : QDialog(p)
{
    QPushButton* btnok = new QPushButton("OK");
    QPushButton* btncancel = new QPushButton("Cancel");

    connect(btnok, SIGNAL(clicked()), this, SLOT(accept()));
    connect(btncancel, SIGNAL(clicked()), this, SLOT(reject()));

    QHBoxLayout* btnlay = new QHBoxLayout();
    btnlay->addWidget(btnok);
    btnlay->addWidget(btncancel);

    QStackedLayout* sl = new QStackedLayout();
    sl->addWidget( createProjectLayout() );
    //sl->addWidget( createReconstructionLayout() );

    QVBoxLayout* lay = new QVBoxLayout();
    lay->addLayout(sl);
    lay->addLayout(btnlay);

    setLayout(lay);
    setWindowTitle("Open Reconstruction");
}

QWidget* OpenReconstructionDialog::createProjectLayout()
{
    mProjectPath = new PathWidget(PathWidget::GET_EXISTING_DIRECTORY);

    QFormLayout* f = new QFormLayout();
    f->addRow("Project:", mProjectPath);

    QWidget* ret = new QWidget;
    ret->setLayout(f);

    return ret;
}

QWidget* OpenReconstructionDialog::createReconstructionLayout()
{
    QVBoxLayout* lay = new QVBoxLayout();

    QWidget* ret = new QWidget;
    ret->setLayout(lay);

    return ret;
}

void OpenReconstructionDialog::onOK()
{
}

void OpenReconstructionDialog::onCancel()
{
}

