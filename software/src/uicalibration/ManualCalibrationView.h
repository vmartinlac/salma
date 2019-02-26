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

    void enumerateFramesWithData(
        std::vector<int>& list);

public:

    enum Mode
    {
        MODE_LEFT,
        MODE_RIGHT,
        MODE_STEREO,
        MODE_PHOTOMETRIC
    };

signals:

    void listOfFramesWithDataChanged();

public slots:

    void home();
    void clear();
    void setFrame(int frame);
    void autoDetect();
    void setMode(Mode mode);

protected:

    void mousePressEvent(QMouseEvent* ev) override;
    void mouseReleaseEvent(QMouseEvent* ev) override;
    void paintEvent(QPaintEvent* ev) override;
    void wheelEvent(QWheelEvent* ev) override;
    void mouseMoveEvent(QMouseEvent* ev) override;

    /*
    void addPoint(const cv::Point2f& pt);
    void removePoint(int id);
    int locatePoint(const QPoint& pt);
    void toggleConnection(int corner1, int corner2);
    */

protected:

    struct ZoomData
    {
        ZoomData();

        void init(QWidget* win, const QImage& img);

        bool valid;
        double factor;
        cv::Point2f point;
    };

    struct CameraFrameData
    {
        std::vector<cv::Point2f> image_points;
        std::vector<cv::Point3f> object_points;
    };

    typedef std::shared_ptr<CameraFrameData> CameraFrameDataPtr;

    struct StereoFrameData
    {
        std::vector< std::pair<cv::Point2f,cv::Point2f> > correspondances;
    };

    typedef std::shared_ptr<StereoFrameData> StereoFrameDataPtr;

    struct PhotometricFrameData
    {
        bool take;
    };

    typedef std::shared_ptr<PhotometricFrameData> PhotometricFrameDataPtr;

protected:

    ManualCalibrationParametersPtr mParams;
    RecordingReaderPtr mReader;

    int mCurrentFrameId;

    Image mCurrentImage;
    QImage mCurrentImageQt;
    cv::Rect mLeftROI;
    cv::Rect mRightROI;

    ZoomData mZoom;

    Mode mMode;

    QPoint mLastMousePosition;

    std::map<int,CameraFrameDataPtr> mLeftCameraData;
    std::map<int,CameraFrameDataPtr> mRightCameraData;
    std::map<int,StereoFrameDataPtr> mStereoData;
    std::map<int,PhotometricFrameDataPtr> mPhotometricData;
};

