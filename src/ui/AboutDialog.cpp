#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QFormLayout>
#include <BuildInfo.h>
#include "AboutDialog.h"

AboutDialog::AboutDialog(QWidget* parent) : QDialog(parent)
{
    QLabel* wtitle = new QLabel("<h1>SALM</h1>");
    QLabel* wdescription = new QLabel("This software was written by Victor MARTIN LAC\nbetween august 2018 and october 2018.");

    const std::string version =
        std::to_string(BuildInfo::getVersionMajor()) + "." +
        std::to_string(BuildInfo::getVersionMinor()) + "." +
        std::to_string(BuildInfo::getVersionRevision());

    QFormLayout* form = new QFormLayout();
    form->addRow("Version :", new QLabel(version.c_str()));
    form->addRow("Compilation date:", new QLabel(BuildInfo::getCompilationDate().c_str()));
    form->addRow("Compiler :", new QLabel(BuildInfo::getCompilerName().c_str()));
    form->addRow("Build type :", new QLabel(BuildInfo::getBuildType().c_str()));

    QPushButton* btn = new QPushButton("Close");

    QVBoxLayout* lay = new QVBoxLayout();
    lay->addWidget(wtitle);
    lay->addWidget(wdescription);
    lay->addLayout(form);
    lay->addWidget(btn);

    setLayout(lay);
    setWindowTitle("About");

    QObject::connect(btn, SIGNAL(clicked()), this, SLOT(accept()));
}

