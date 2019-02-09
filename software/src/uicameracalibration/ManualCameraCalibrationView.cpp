#include <opencv2/imgproc.hpp>
#include <iostream>
#include <QPainter>
#include <QPaintEvent>
#include <QWheelEvent>
#include <QMouseEvent>
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

void ManualCameraCalibrationView::wheelEvent(QWheelEvent* ev)
{
    const double factor_max = 20.0;
    const double factor_min = 1.0/20.0;
    const double speed = 2.0e-3;

    if(mZoom.valid)
    {
        const QPoint delta = ev->angleDelta();

        if(delta.isNull() == false)
        {
            const double new_factor = mZoom.factor * std::exp(speed * delta.y());
            mZoom.factor = std::max(factor_min, std::min(factor_max, new_factor));
            update();
        }
    }

    mLastMousePosition = ev->pos();

    ev->accept();
}

void ManualCameraCalibrationView::mouseReleaseEvent(QMouseEvent* ev)
{
    mLastMousePosition = ev->pos();

    ev->accept();
}

void ManualCameraCalibrationView::mousePressEvent(QMouseEvent* ev)
{
    mLastMousePosition = ev->pos();

    ev->accept();
}

void ManualCameraCalibrationView::mouseMoveEvent(QMouseEvent* ev)
{
    if(ev->buttons() & Qt::RightButton)
    {
        if(mZoom.valid)
        {
            const QPoint delta = ev->pos() - mLastMousePosition;
            mZoom.point.x -= delta.x() / mZoom.factor;
            mZoom.point.y -= delta.y() / mZoom.factor;
            update();
        }
    }

    mLastMousePosition = ev->pos();

    ev->accept();
}

void ManualCameraCalibrationView::paintEvent(QPaintEvent* ev)
{
    QPainter p(this);

    p.setBackground(QColor(Qt::black));
    p.eraseRect(rect());

    if(mFrame.isNull() == false)
    {
        if(mZoom.valid == false)
        {
            mZoom.init(mFrame);
        }

        cv::Rect source(0,0,mFrame.width(), mFrame.height());

        cv::Rect destination(
            width()/2 - mZoom.factor*mZoom.point.x,
            height()/2 - mZoom.factor*mZoom.point.y,
            mFrame.width()*mZoom.factor,
            mFrame.height()*mZoom.factor);

        p.drawImage(
            QRect(destination.x, destination.y, destination.width, destination.height),
            mFrame,
            QRect(source.x, source.y, source.width, source.height));
    }

    ev->accept();
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

    update();
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

