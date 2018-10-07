#pragma once

#include <QWidget>
#include <opencv2/core.hpp>
#include "Port.h"

struct VideoInputData
{
    cv::Mat image;
};

typedef Port<VideoInputData> VideoInputPort;

class VideoWidget : public QWidget
{
    Q_OBJECT

public:

    VideoWidget(QWidget* parent=nullptr);

    VideoInputPort* getPort();

protected:

    void paintEvent(QPaintEvent*) override;

protected slots:

    void refresh();

protected:

    VideoInputPort* mPort;
    QImage mImage;
};

