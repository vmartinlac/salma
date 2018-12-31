
#include <QVBoxLayout>
#include <QSettings>
#include <QMessageBox>
#include <QDoubleValidator>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QPushButton>
#include "NewReconstructionDialog.h"
#include "VideoSystem.h"
#include "ReconstructionOperation.h"

NewReconstructionDialog::NewReconstructionDialog(Project* proj, QWidget* parent) : NewOperationDialog(proj, parent)
{
    mName = new QLineEdit();

    QFormLayout* form = new QFormLayout();
    form->addRow("Name:", mName);

    QPushButton* btnok = new QPushButton("OK");
    QPushButton* btncancel = new QPushButton("Cancel");

    QHBoxLayout* hlay = new QHBoxLayout();
    hlay->addWidget(btnok);
    hlay->addWidget(btncancel);

    QVBoxLayout* vlay = new QVBoxLayout();
    vlay->addLayout(form);
    vlay->addLayout(hlay);

    setLayout(vlay);
    setWindowTitle("New Reconstruction");

    connect(btnok, SIGNAL(clicked()), this, SLOT(accept()));
    connect(btncancel, SIGNAL(clicked()), this, SLOT(reject()));
}

void NewReconstructionDialog::accept()
{
    QString name;

    OperationPtr op;
    bool ok = true;
    const char* err = "";

    if(ok)
    {
        name = mName->text();
        ok = (name.isEmpty() == false);
        err = "Incorrect name!";
    }

    if(ok)
    {
        ReconstructionOperation* myop = new ReconstructionOperation();
        myop->mReconstructionName = mName->text().toStdString();
        op.reset(myop);
    }

    if(ok)
    {
        setOperation(op);
        QDialog::accept();
    }
    else
    {
        QMessageBox::critical(this, "Error", err);
    }
}

