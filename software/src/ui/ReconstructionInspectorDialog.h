#pragma once

#include <QComboBox>
//#include <QLabel>
#include <QDialog>
#include "SLAMDataStructures.h"
#include "ViewerWidget.h"

class ReconstructionInspectorDialog : public QDialog
{
    Q_OBJECT

public:

    ReconstructionInspectorDialog(SLAMReconstructionPtr reconstr, QWidget* parent=nullptr);

protected slots:

    void onExport();
    void onSelectSegment();

protected:

    SLAMReconstructionPtr mReconstruction;
    ViewerWidget* mViewer;
    QComboBox* mSegmentsWidget;
    //QLabel* mInformation;
};

