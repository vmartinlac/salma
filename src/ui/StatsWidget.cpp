#include <iostream>
#include <QFormLayout>
#include "StatsWidget.h"

StatsWidget::StatsWidget(
    SLAMOutput* slam,
    QWidget* parent) :
    m_slam(slam),
    QWidget(parent)
{
    QFormLayout* lay = new QFormLayout();

    m_mode = new QLabel("n/a");
    lay->addRow("Mode:", m_mode);

    m_frame_id = new QLabel("n/a");
    lay->addRow("Frame ID:", m_frame_id);

    m_timestamp = new QLabel("n/a");
    lay->addRow("Timestamp:", m_timestamp);

    m_position = new QLabel("n/a");
    lay->addRow("Position:", m_position);

    m_attitude = new QLabel("n/a");
    lay->addRow("Attitude:", m_attitude);

    m_linear_velocity = new QLabel("n/a");
    lay->addRow("Linear velocity:", m_linear_velocity);

    m_angular_velocity = new QLabel("n/a");
    lay->addRow("Angular velocity:", m_angular_velocity);

    m_num_landmarks = new QLabel("n/a");
    lay->addRow("Number of landmarks:", m_num_landmarks);

    setLayout(lay);

    connect(slam, SIGNAL(updated()), this, SLOT(refresh()), Qt::QueuedConnection);
}

static QString eigenToQt(const Eigen::Quaterniond& vec)
{
    return
        "{ w = " + QString::number(vec.w()) +
        " ; x = " + QString::number(vec.x()) +
        " ; y = " + QString::number(vec.y()) +
        " ; z = " + QString::number(vec.z()) + " }";
}

static QString eigenToQt(const Eigen::Vector3d& vec)
{
    return
        "{ x = " + QString::number(vec.x()) +
        " ; y = " + QString::number(vec.y()) +
        " ; z = " + QString::number(vec.z()) + " }";
}

void StatsWidget::refresh()
{
    m_slam->beginRead();
    m_mode->setText( m_slam->mode );
    m_timestamp->setText( QString::number(m_slam->timestamp) );
    m_frame_id->setText( QString::number(m_slam->frame_id) );
    m_position->setText( eigenToQt(m_slam->position) );
    m_attitude->setText( eigenToQt(m_slam->attitude) );
    m_linear_velocity->setText( eigenToQt(m_slam->linear_velocity) );
    m_angular_velocity->setText( eigenToQt(m_slam->angular_velocity) );
    m_num_landmarks->setText( QString::number(m_slam->landmarks.size()) );
    m_slam->endRead();
}

