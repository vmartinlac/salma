#include <QVBoxLayout>
#include <QScrollArea>
#include <QImage>
#include <QPixmap>
#include <QLabel>
#include "VideoWidget.h"

VideoWidget::VideoWidget(SLAMOutput* slam, QWidget* parent) :
    m_slam(slam),
    QWidget(parent)
{

    QImage image(640, 480, QImage::Format_RGB888);
    image.fill(Qt::black);

    QLabel* l = new QLabel();
    l->setPixmap(QPixmap::fromImage(image));

    QScrollArea* s = new QScrollArea();
    s->setAlignment(Qt::AlignCenter);
    s->setWidget(l);

    QVBoxLayout* lay = new QVBoxLayout();
    lay->addWidget(s);

    setLayout(lay);
}
