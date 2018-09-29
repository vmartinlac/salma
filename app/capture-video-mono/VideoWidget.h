#pragma once

#include <QWidget>

class VideoWidget : public QWidget
{
public:

    VideoWidget(QWidget* parent=nullptr);

protected:

    void paintEvent(QPaintEvent*) override;
};

