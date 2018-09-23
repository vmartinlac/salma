#include <QPainter>
#include <QVBoxLayout>
#include <QScrollArea>
#include <QImage>
#include <QPixmap>
#include <QLabel>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include "VideoWidget.h"

VideoWidget::VideoWidget(SLAMOutput* slam, QWidget* parent) :
    m_slam(slam),
    QWidget(parent)
{

    QImage image(640, 480, QImage::Format_RGB888);
    image.fill(Qt::black);

    /*
    QPainter p(&image);
    p.setPen(Qt::NoPen);
    p.setBrush(Qt::red);
    p.drawEllipse(QPoint(320, 240), 100, 100);
    p.setBrush(Qt::green);
    p.drawEllipse(QPoint(320, 240), 75, 75);
    p.setBrush(Qt::blue);
    p.drawEllipse(QPoint(320, 240), 50, 50);
    p.setBrush(Qt::yellow);
    p.drawEllipse(QPoint(320, 240), 25, 25);
    */

    m_label = new QLabel();
    m_label->setScaledContents(true);
    m_label->resize(640, 480);
    m_label->setPixmap(QPixmap::fromImage(image));

    QScrollArea* s = new QScrollArea();
    s->setAlignment(Qt::AlignCenter);
    s->setWidget(m_label);

    QVBoxLayout* lay = new QVBoxLayout();
    lay->addWidget(s);

    setLayout(lay);

    connect(slam, SIGNAL(updated()), this, SLOT(refresh()), Qt::QueuedConnection);
}

void VideoWidget::refresh()
{
    m_slam->beginRead();

    if( m_slam->image.type() != CV_8UC3 ) throw std::runtime_error("internal error");

    cv::Mat rgb_converted;
    cv::cvtColor( m_slam->image, rgb_converted, cv::COLOR_BGR2RGB );

    QImage image(
        rgb_converted.ptr(),
        rgb_converted.cols,
        rgb_converted.rows,
        static_cast<int>(rgb_converted.step),
        QImage::Format_RGB888);

    m_label->setPixmap( QPixmap::fromImage(image) );
    //m_label->resize( image.width(), image.height() );

    m_slam->endRead();
}

