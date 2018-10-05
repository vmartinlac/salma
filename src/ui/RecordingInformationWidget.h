#pragma once

#include <QWidget>
#include <QLabel>
#include "RecordingOutput.h"

class RecordingInformationWidget : public QWidget
{
    Q_OBJECT

public:

    RecordingInformationWidget(RecordingOutput* output, QWidget* parent=nullptr);

protected slots:

    void refresh();

protected:

    QLabel* mLabelCamera;
    QLabel* mLabelOutputDirectory;
    QLabel* mLabelNumFrames;
    QLabel* mLabelResolution;
    RecordingOutput* mOutput;
};

