#pragma once

#include <QLabel>
#include <QDialog>
#include <QDoubleSpinBox>
#include <QSpinBox>

class TargetParametersDialog : public QDialog
{
    Q_OBJECT
public:

    TargetParametersDialog(QWidget* parent=nullptr);

    double getCellLength();

protected slots:

    void updateCellSizeLabel();
    void accept() override;

protected:

    QDoubleSpinBox* mLength;
    QSpinBox* mNumberOfCells;
    QLabel* mCellSize;
};

