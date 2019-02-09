#pragma once

#include <QWidget>
#include <opencv2/core.hpp>
#include "ManualCameraCalibrationParameters.h"
#include "RecordingReader.h"

class ManualCameraCalibrationView : public QWidget
{
    Q_OBJECT

public:

    ManualCameraCalibrationView(
        ManualCameraCalibrationParametersPtr params,
        QWidget* parent=nullptr);

    ~ManualCameraCalibrationView();

public slots:

    void home();
    void setFrame(int frame);

protected:

    void mousePressEvent(QMouseEvent* ev) override;
    void mouseReleaseEvent(QMouseEvent* ev) override;
    void paintEvent(QPaintEvent* ev) override;
    void wheelEvent(QWheelEvent* ev) override;
    void mouseMoveEvent(QMouseEvent* ev) override;

protected:

    struct ZoomData
    {
        ZoomData();

        void init(const QImage& img);

        bool valid;
        double factor;
        cv::Point2f point;
    };

protected:

    ManualCameraCalibrationParametersPtr mParams;
    RecordingReaderPtr mReader;

    //std::map<int,cv::Point2f> mCorners;
    //std::vector< std::pair<int,int> > mEdges;

    QImage mFrame;
    ZoomData mZoom;

    QPoint mLastMousePosition;
};

