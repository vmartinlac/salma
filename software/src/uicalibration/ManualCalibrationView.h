#pragma once

#include <QWidget>
#include <array>
#include <opencv2/core.hpp>
#include "ManualCalibrationParameters.h"
#include "RecordingReader.h"

class ManualCalibrationView : public QWidget
{
    Q_OBJECT

public:

    ManualCalibrationView(
        ManualCalibrationParametersPtr params,
        QWidget* parent=nullptr);

    ~ManualCalibrationView();

    bool getCalibrationData(
        std::vector< std::vector<cv::Point2f> >& image_points,
        std::vector< std::vector<cv::Point3f> >& object_points,
        cv::Size& size);

public:

    enum Mode
    {
        MODE_LEFT,
        MODE_RIGHT,
        MODE_STEREO,
        MODE_PHOTOMETRIC
    };

public slots:

    void home();
    void clear();
    void setFrame(int frame);
    void autoDetect();
    void propagate();
    void setMode(Mode mode);

protected:

    void mousePressEvent(QMouseEvent* ev) override;
    void mouseReleaseEvent(QMouseEvent* ev) override;
    void paintEvent(QPaintEvent* ev) override;
    void wheelEvent(QWheelEvent* ev) override;
    void mouseMoveEvent(QMouseEvent* ev) override;

    void addPoint(const cv::Point2f& pt);
    void removePoint(int id);
    int locatePoint(const QPoint& pt);
    void toggleConnection(int corner1, int corner2);

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

        cv::Point2f image_coords;

        bool has_object_coords;
        cv::Point2i object_coords;

        std::array<int,4> neighbors;

        // only for graph algorithms.
        bool bfs_visited;
        bool bfs_has_coords;
        cv::Point2i bfs_coords;
    };

    struct FrameData
    {
        std::map<int,FramePoint> corners;
    };

protected:

    ManualCalibrationParametersPtr mParams;
    RecordingReaderPtr mReader;

    int mCurrentFrameId;
    QImage mFrame;
    cv::Mat mFrameOpenCV;
    ZoomData mZoom;
    Mode mMode;
    int mSelectedPoint;

    QPoint mLastMousePosition;

    std::map<int,FrameData> mFrameData;
};

