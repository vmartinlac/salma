#pragma once

#include <QLineEdit>

class TargetScaleWidget : public QLineEdit
{
public:

    TargetScaleWidget(QWidget* parent=nullptr);
    ~TargetScaleWidget();

    double getScale(bool& ok);
};

