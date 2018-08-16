#pragma once

#include <QWidget>
#include "SLAMOutput.h"

class VideoWidget : public QWidget
{
    Q_OBJECT

public:

    VideoWidget(
        SLAMOutput* slam,
        QWidget* parent=nullptr);

protected:

    SLAMOutput* m_slam;
};
