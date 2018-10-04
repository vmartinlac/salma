#pragma once

#include <QWidget>
#include "Output.h"

class VideoWidget : public QWidget
{
    Q_OBJECT

public:

    VideoWidget(Output* output, QWidget* parent=nullptr);

protected:

    void paintEvent(QPaintEvent*) override;

protected slots:

    void refresh();

protected:

    Output* mOutput;
    QImage mImage;
};

