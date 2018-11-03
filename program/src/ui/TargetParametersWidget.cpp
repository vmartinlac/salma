#include <QPushButton>
#include <QHBoxLayout>
#include "TargetParametersDialog.h"
#include "TargetParametersWidget.h"

TargetParametersWidget::TargetParametersWidget(QWidget* parent) : QWidget(parent)
{
    mCellLength = new QDoubleSpinBox();
    mCellLength->setDecimals(5);
    mCellLength->setMinimum(0.0);
    mCellLength->setValue(1.0);

    QPushButton* btn = new QPushButton("change");

    QHBoxLayout* l = new QHBoxLayout();
    l->setContentsMargins(0, 0, 0, 0);
    l->addWidget(mCellLength, 1);
    l->addWidget(btn, 0);

    setLayout(l);

    QObject::connect(btn, SIGNAL(clicked()), this, SLOT(changeCellLength()));
}

void TargetParametersWidget::changeCellLength()
{
    TargetParametersDialog* dlg = new TargetParametersDialog(this);
    const int ret = dlg->exec();
    if(ret == QDialog::Accepted)
    {
        mCellLength->setValue( dlg->getCellLength() );
    }
}

double TargetParametersWidget::getCellLength()
{
    return mCellLength->value();
}

