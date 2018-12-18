#pragma once

#include <QDialog>
#include <QListWidget>
#include <memory>
#include "SLAMReconstructionDB.h"
#include "PathWidget.h"
#include "VisualizationData.h"
#include "VisualizationSettings.h"

class OpenReconstructionDialog : public QDialog
{
    Q_OBJECT

public:

    OpenReconstructionDialog(VisualizationDataPort* visu_data, VisualizationSettingsPort* settings, QWidget* p=nullptr);

protected slots:

    void openDatabase();
    void accept() override;

protected:

    std::shared_ptr<SLAMReconstructionDB> mDB;
    PathWidget* mProjectPath;
    QListWidget* mReconstructionList;
    VisualizationDataPort* mVisualizationData;
    VisualizationSettingsPort* mVisualizationSettings;
};

