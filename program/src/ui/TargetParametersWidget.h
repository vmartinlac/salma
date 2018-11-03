#pragma once

#include <QWidget>
#include <QDoubleSpinBox>

class TargetParametersWidget : public QWidget
{
    Q_OBJECT

public:

    TargetParametersWidget(QWidget* parent=nullptr);

    double getCellLength();

protected slots:

    void changeCellLength();

protected:

    QDoubleSpinBox* mCellLength;
};

