#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include "VisualizationSettingsDialog.h"

VisualizationSettingsDialog::VisualizationSettingsDialog(QWidget* p) : QDialog(p)
{
    mShowRig = new QCheckBox("Show rig");
    mShowTrajectory = new QCheckBox("Show trajectory");
    mShowMapPoints = new QCheckBox("Show map points");
    mShowDensePoints = new QCheckBox("Show dense points");

    QPushButton* btnok = new QPushButton("OK");
    QPushButton* btncancel = new QPushButton("Cancel");

    connect(btnok, SIGNAL(clicked()), this, SLOT(accept()));
    connect(btncancel, SIGNAL(clicked()), this, SLOT(reject()));

    QHBoxLayout* btnlay = new QHBoxLayout();
    btnlay->addWidget(btnok);
    btnlay->addWidget(btncancel);

    QVBoxLayout* l = new QVBoxLayout();
    l->addWidget(mShowRig);
    l->addWidget(mShowTrajectory);
    l->addWidget(mShowMapPoints);
    l->addWidget(mShowDensePoints);
    l->addLayout(btnlay);

    setLayout(l);
    setWindowTitle("Visualization Settings");
}

