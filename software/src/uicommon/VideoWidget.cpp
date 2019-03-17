#include <QPainter>
#include <opencv2/imgproc.hpp>
#include "VideoWidget.h"

VideoWidget::VideoWidget(QWidget* parent) : QWidget(parent)
{
    mPort = new VideoInputPort(this);

    QObject::connect(mPort, SIGNAL(updated()), this, SLOT(refresh()));

    mImage = QImage(320, 200, QImage::Format_RGB888);
    mImage.fill(Qt::black);

    setMinimumSize(320, 200);
}

VideoInputPort* VideoWidget::getPort()
{
    return mPort;
}

void VideoWidget::paintEvent(QPaintEvent*)
{
    QPainter painter(this);

    painter.fillRect(rect(), Qt::black);

    painter.drawImage(
        QPoint(width()/2 - mImage.width()/2, height()/2 - mImage.height()/2),
        mImage);
}

void VideoWidget::refresh()
{
    const int target_width = 600;

    VideoInputData data;
    mPort->read(data);

    cv::Mat small;
    if( data.image.cols > target_width )
    {
        cv::resize( data.image, small, cv::Size(target_width, data.image.rows*target_width/data.image.cols), cv::INTER_NEAREST );
    }
    else
    {
        small = data.image;
    }

    QImage img;

    if( small.type() == CV_8UC3 )
    {
        cv::Mat rgb;
        cv::cvtColor(small, rgb, cv::COLOR_BGR2RGB);

        QImage img(
            rgb.data,
            rgb.cols,
            rgb.rows,
            rgb.step,
            QImage::Format_RGB888);

        mImage = img.copy();
    }
    else if( small.type() == CV_8UC1 )
    {
        QImage img(
            small.data,
            small.cols,
            small.rows,
            small.step,
            QImage::Format_Grayscale8);

        mImage = img.copy();
    }
    else
    {
        throw std::runtime_error("VideoWidget received frame with incorrect format!");
    }

    resize(mImage.width(), mImage.height());

    update();
}

