#include <QPainter>
#include "VideoWidget.h"

VideoWidget::VideoWidget(QWidget* parent) : QWidget(parent)
{
    setMinimumSize(320, 200);
}

void VideoWidget::paintEvent(QPaintEvent*)
{
    QPainter painter(this);

    painter.fillRect(rect(), Qt::black);
}
