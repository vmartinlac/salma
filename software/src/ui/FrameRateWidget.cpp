#include "FrameRateWidget.h"

FrameRateWidget::FrameRateWidget(QWidget* parent) : QDoubleSpinBox(parent)
{
    setDecimals(2);
    setMinimum(0.0);
    setMaximum(100.0);
    setSingleStep(0.1);
    setValue(15.0);
}

FrameRateWidget::~FrameRateWidget()
{
}

double FrameRateWidget::getFrameRate()
{
    return value();
}

