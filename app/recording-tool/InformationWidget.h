#pragma once

#include <QWidget>
#include <QLabel>
#include "Output.h"

class InformationWidget : public QWidget
{
    Q_OBJECT

public:

    InformationWidget(Output* output, QWidget* parent=nullptr);

protected slots:

    void refresh();

protected:

    QLabel* mLabelCamera;
    QLabel* mLabelOutputDirectory;
    QLabel* mLabelNumFrames;
    QLabel* mLabelResolution;
    Output* mOutput;
};

