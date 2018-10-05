#pragma once

#include <QWidget>
#include "RecordingOutput.h"

class RecordingVideoWidget : public QWidget
{
    Q_OBJECT

public:

    RecordingVideoWidget(RecordingOutput* output, QWidget* parent=nullptr);

protected:

    void paintEvent(QPaintEvent*) override;

protected slots:

    void refresh();

protected:

    RecordingOutput* mOutput;
    QImage mImage;
};

