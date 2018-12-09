#pragma once

#include <QDialog>
#include <QCheckBox>

class VisualizationSettingsDialog : public QDialog
{
    Q_OBJECT

public:

    VisualizationSettingsDialog(QWidget* parent=nullptr);

protected:

    QCheckBox* mShowRig;
    QCheckBox* mShowTrajectory;
    QCheckBox* mShowMapPoints;
    QCheckBox* mShowDensePoints;
};

