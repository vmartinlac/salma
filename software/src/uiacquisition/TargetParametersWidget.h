#pragma once

#include <QWidget>
#include <QDoubleSpinBox>

class TargetParametersWidget : public QWidget
{
    Q_OBJECT

public:

    TargetParametersWidget(QWidget* parent=nullptr);

    void setCellLength(double value);
    double getCellLength();

protected slots:

    void changeCellLength();

protected:

    QDoubleSpinBox* mCellLength;
};

