#pragma once

#include <QWidget>
#include <QLabel>
#include "SLAMOutput.h"

class VideoWidget : public QWidget
{
    Q_OBJECT

public:

    VideoWidget(
        SLAMOutput* slam,
        QWidget* parent=nullptr);

protected slots:

    void refresh();

protected:

    SLAMOutput* m_slam;
    QLabel* m_label;
};
