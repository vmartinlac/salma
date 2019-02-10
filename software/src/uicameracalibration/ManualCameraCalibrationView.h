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

    bool getCalibrationData(
        std::vector< std::vector<cv::Point2f> >& image_points,
        std::vector< std::vector<cv::Point3f> >& object_points,
        cv::Size& size);

public:

    enum Mode
    {
        MODE_CORNER,
        MODE_CONNECTION
    };

public slots:

    void home();
    void clear();
    void setFrame(int frame);
    void setMode(Mode mode);
    void setModeToCorner();
    void setModeToConnection();
    void autoDetect();

protected:

    void mousePressEvent(QMouseEvent* ev) override;
    void mouseReleaseEvent(QMouseEvent* ev) override;
    void paintEvent(QPaintEvent* ev) override;
    void wheelEvent(QWheelEvent* ev) override;
    void mouseMoveEvent(QMouseEvent* ev) override;

    void addPoint(const cv::Point2f& pt);
    void removePoint(int id);
    int locatePoint(const QPoint& pt);

protected:

    struct ZoomData
    {
        ZoomData();

        void init(const QImage& img);

        bool valid;
        double factor;
        cv::Point2f point;
    };

    struct FramePoint
    {
        FramePoint();

        bool has_object_point;
        cv::Point2f image_point;;
        cv::Point3f object_point;
    };

    struct FrameData
    {
        std::map<int,FramePoint> corners;
        std::vector< std::pair<int,int> > edges;
    };

protected:

    ManualCameraCalibrationParametersPtr mParams;
    RecordingReaderPtr mReader;

    int mCurrentFrameId;
    QImage mFrame;
    cv::Mat mFrameOpenCV;
    ZoomData mZoom;
    Mode mMode;

    QPoint mLastMousePosition;

    std::map<int,FrameData> mFrameData;
};

