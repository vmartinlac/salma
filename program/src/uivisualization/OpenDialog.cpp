#include <QVBoxLayout>
#include <QTextEdit>
#include <QHBoxLayout>
#include <QPushButton>
#include "OpenDialog.h"
#include "PathWidget.h"

OpenDialog::OpenDialog(QWidget* w) : QDialog(w)
{
    QPushButton* btnok = new QPushButton("OK");
    QPushButton* btncancel = new QPushButton("Cancel");

    QHBoxLayout* btnlay = new QHBoxLayout();
    btnlay->addWidget(btnok);
    btnlay->addWidget(btncancel);

    QVBoxLayout* mainlay = new QVBoxLayout();
    mainlay->addWidget(new QTextEdit());
    mainlay->addLayout(btnlay);

    setWindowTitle("Open Reconstruction");
}

