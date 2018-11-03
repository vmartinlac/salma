#include <QSettings>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QFormLayout>
#include "TargetParametersDialog.h"

TargetParametersDialog::TargetParametersDialog(QWidget* parent) : QDialog(parent)
{
    mLength = new QDoubleSpinBox();
    mNumberOfCells = new QSpinBox();
    mCellSize = new QLabel();

    mLength->setDecimals(5);
    mLength->setMinimum(0.0);
    mLength->setValue(1.0);

    mNumberOfCells->setMinimum(1);
    mNumberOfCells->setValue(1);

    QFormLayout* l = new QFormLayout();
    l->addRow("Number of cells", mNumberOfCells);
    l->addRow("Length", mLength);
    l->addRow("Cell size", mCellSize);

    QPushButton* btnok = new QPushButton("OK");
    QPushButton* btncancel = new QPushButton("Cancel");

    QHBoxLayout* hl = new QHBoxLayout();
    hl->addWidget(btnok);
    hl->addWidget(btncancel);

    QVBoxLayout* vl = new QVBoxLayout();
    vl->addLayout(l);
    vl->addLayout(hl);

    setLayout(vl);
    setWindowTitle("Target parameters");

    QObject::connect(mNumberOfCells, SIGNAL(valueChanged(int)), this, SLOT(updateCellSizeLabel()));
    QObject::connect(mLength, SIGNAL(valueChanged(double)), this, SLOT(updateCellSizeLabel()));
    QObject::connect(btnok, SIGNAL(clicked()), this, SLOT(accept()));
    QObject::connect(btncancel, SIGNAL(clicked()), this, SLOT(reject()));

    QSettings s;
    s.beginGroup("target_parameters");
    mNumberOfCells->setValue( s.value("number_of_cells", 1.0).toInt() );
    mLength->setValue( s.value("length", 1.0).toDouble() );
    s.endGroup();

    updateCellSizeLabel();
};

double TargetParametersDialog::getCellLength()
{
    return mLength->value() / double(mNumberOfCells->value());
}

void TargetParametersDialog::updateCellSizeLabel()
{
    mCellSize->setText(QString::number(getCellLength()));
}

void TargetParametersDialog::accept()
{
    QSettings s;

    s.beginGroup("target_parameters");
    s.setValue("number_of_cells", mNumberOfCells->value());
    s.setValue("length", mLength->value());
    s.endGroup();

    s.sync();

    QDialog::accept();
}

