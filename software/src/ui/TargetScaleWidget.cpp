#include <QDoubleValidator>
#include "TargetScaleWidget.h"

TargetScaleWidget::TargetScaleWidget(QWidget* parent) : QLineEdit(parent)
{
    QDoubleValidator* val = new QDoubleValidator();
    val->setBottom(1.0e-6);
    setValidator(val);
    setText(QString::number(1.0));
}

TargetScaleWidget::~TargetScaleWidget()
{
}

double TargetScaleWidget::getScale(bool& ok)
{
    return text().toInt(&ok);
}

