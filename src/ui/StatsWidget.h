#pragma once

#include <QWidget>
#include <QLabel>
#include "SLAMOutput.h"

class StatsWidget : public QWidget
{
    Q_OBJECT

public:

    StatsWidget(
        SLAMOutput* slam,
        QWidget* parent=nullptr);

protected slots:

    void refresh();

protected:

    QLabel* m_mode;
    QLabel* m_frame_id;
    QLabel* m_position;
    QLabel* m_attitude;
    QLabel* m_linear_velocity;
    QLabel* m_angular_velocity;
    QLabel* m_num_landmarks;
    QLabel* m_timestamp;
    SLAMOutput* m_slam;
};
