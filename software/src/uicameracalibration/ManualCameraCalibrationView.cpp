#include <opencv2/imgproc.hpp>
#include <iostream>
#include <QPainter>
#include <QPaintEvent>
#include "RecordingReader.h"
#include "ManualCameraCalibrationView.h"

ManualCameraCalibrationView::ManualCameraCalibrationView(
    ManualCameraCalibrationParametersPtr params,
    QWidget* parent) : QWidget(parent)
{
    mParams = params;
    mReader.reset(new RecordingReader(mParams->recording, true));

    mReader->open();

    setMinimumSize(320, 200);
}

ManualCameraCalibrationView::~ManualCameraCalibrationView()
{
    mReader->close();
}

void ManualCameraCalibrationView::paintEvent(QPaintEvent* ev)
{
    QPainter p(this);

    if(mFrame.isNull())
    {
        p.fillRect(rect(), QColor(Qt::black));
    }
    else
    {
        if(mZoom.valid == false)
        {
            mZoom.init(mFrame);
        }

        cv::Rect destination(0,0,width(), height());
        cv::Rect source(0,0,mFrame.width(), mFrame.height());

        p.drawImage(
            QRect(destination.x, destination.y, destination.width, destination.height),
            mFrame,
            QRect(source.x, source.y, source.width, source.height));
    }
}

void ManualCameraCalibrationView::setFrame(int frame)
{
    mReader->seek(frame);

    mReader->trigger();

    Image img;
    mReader->read(img);

    if(img.isValid())
    {
        cv::Mat rgb;
        cv::cvtColor(img.getFrame(), rgb, cv::COLOR_BGR2RGB);

        QImage img(
            rgb.data,
            rgb.cols,
            rgb.rows,
            rgb.step,
            QImage::Format_RGB888);

        mFrame = img.copy();

        if(mZoom.valid == false)
        {
            mZoom.init(mFrame);
        }
    }
    else
    {
        mFrame = QImage();
        std::cerr << "Invalid frame!" << std::endl;
    }

    update();
}

void ManualCameraCalibrationView::home()
{
    if(mFrame.isNull())
    {
        mZoom.valid = false;
    }
    else
    {
        mZoom.init(mFrame);
    }
}

ManualCameraCalibrationView::ZoomData::ZoomData()
{
    valid = false;
    factor = 1.0;
}

void ManualCameraCalibrationView::ZoomData::init(const QImage& img)
{
    valid = true;
    factor = 1.0;
    point.x = img.width() / 2;
    point.y = img.height() / 2;
}

