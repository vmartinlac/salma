#pragma once

#include <QDoubleSpinBox>

class FrameRateWidget : public QDoubleSpinBox
{
public:

    FrameRateWidget(QWidget* parent=nullptr);
    ~FrameRateWidget();

    double getFrameRate();
};

