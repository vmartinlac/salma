#pragma once

#include <QMainWindow>
#include "VideoWidget.h"
#include "InformationWidget.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    MainWindow(QWidget* parent=nullptr);

protected:

    VideoWidget* mVideoWidget;
    InformationWidget* mInformationWidget;

protected slots:

    void configure();
    void startRecording();
    void stopRecording();
    void about();
};

