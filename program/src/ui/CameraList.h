#pragma once

#include <QComboBox>

class CameraList : public QComboBox
{
public:

    CameraList(QWidget* parent=nullptr);

    int getCameraId();

protected:

    void refresh();
};

