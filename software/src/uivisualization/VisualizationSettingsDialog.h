#pragma once

#include <QDialog>
#include <QCheckBox>
#include "VisualizationSettings.h"

class VisualizationSettingsDialog : public QDialog
{
    Q_OBJECT

public:

    VisualizationSettingsDialog(
        VisualizationSettingsPort* visusettings,
        QWidget* parent=nullptr);

protected:

    QCheckBox* mShowRig;
    QCheckBox* mShowTrajectory;
    QCheckBox* mShowMapPoints;
    QCheckBox* mShowDensePoints;
    VisualizationSettingsPort* mVisualizationSettings;
};

