#include <QPainter>
#include <opencv2/imgproc.hpp>
#include "RecordingVideoWidget.h"

RecordingVideoWidget::RecordingVideoWidget(RecordingOutput* output, QWidget* parent) : QWidget(parent)
{
    mOutput = output;
    setMinimumSize(320, 200);

    QObject::connect(output, SIGNAL(updated()), this, SLOT(refresh()));

    mImage = QImage(320, 200, QImage::Format_RGB888);
    mImage.fill(Qt::black);
}

void RecordingVideoWidget::paintEvent(QPaintEvent*)
{
    QPainter painter(this);

    painter.fillRect(rect(), Qt::black);

    painter.drawImage(
        QPoint(width()/2 - mImage.width()/2, height()/2 - mImage.height()/2),
        mImage);
}

void RecordingVideoWidget::refresh()
{
    const int target_width = 400;

    RecordingOutputData data;
    mOutput->read(data);

    cv::Mat small;
    cv::resize( data.image, small, cv::Size(target_width, data.image.rows*target_width/data.image.cols), cv::INTER_NEAREST );

    cv::Mat rgb;
    cv::cvtColor(small, rgb, cv::COLOR_BGR2RGB);

    QImage img(
        rgb.data,
        rgb.cols,
        rgb.rows,
        rgb.step,
        QImage::Format_RGB888);

    mImage = img.copy();

    if(width() < mImage.width() || height() < mImage.height())
    {
        setMinimumSize(mImage.width(), mImage.height());
    }

    update();
}

