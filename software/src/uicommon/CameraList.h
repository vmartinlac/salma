#pragma once

#include <QComboBox>

class CameraList : public QComboBox
{
public:

    CameraList(QWidget* parent=nullptr);

    void setSelectedCamera(const std::string& camid);
    int getCameraId();

protected:

    void refresh();
};

