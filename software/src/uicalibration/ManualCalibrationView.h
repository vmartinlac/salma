#pragma once

#include <QWidget>
#include <array>
#include <opencv2/core.hpp>
#include "CalibrationResiduals.h"
#include "ManualCalibrationParameters.h"
#include "StereoRigCalibration.h"
#include "RecordingReader.h"

class ManualCalibrationView : public QWidget
{
    Q_OBJECT

public:

    ManualCalibrationView(
        ManualCalibrationParametersPtr params,
        QWidget* parent=nullptr);

    ~ManualCalibrationView();

    bool doCalibrate(
        StereoRigCalibrationPtr& calibration,
        CalibrationResiduals& residuals);

    void enumerateFramesWithData(
        std::vector<int>& list);

public:

    enum Mode
    {
        MODE_LEFT,
        MODE_RIGHT,
        MODE_STEREO
    };

signals:

    void listOfFramesWithDataChanged();

public slots:

    void home();
    void setFrame(int frame);
    void setMode(Mode mode);
    void doClear();
    void doTake();

protected:

    struct ZoomData
    {
        ZoomData();

        void init(QWidget* win, const QImage& img);

        bool valid;
        double factor;
        cv::Point2f point;
    };

    struct MonoCorrespondance
    {
        cv::Point3f object_point;
        cv::Point2f image_point;
    };

    struct CameraFrameData
    {
        std::vector<MonoCorrespondance> points;
    };

    typedef std::shared_ptr<CameraFrameData> CameraFrameDataPtr;

    struct StereoCorrespondance
    {
        cv::Point3f object_point;
        cv::Point2f left_image_point;
        cv::Point2f right_image_point;
    };

    struct StereoFrameData
    {
        std::vector<StereoCorrespondance> points;
    };

    typedef std::shared_ptr<StereoFrameData> StereoFrameDataPtr;

protected:

    void mousePressEvent(QMouseEvent* ev) override;
    void mouseReleaseEvent(QMouseEvent* ev) override;
    void paintEvent(QPaintEvent* ev) override;
    void wheelEvent(QWheelEvent* ev) override;
    void mouseMoveEvent(QMouseEvent* ev) override;

    bool askScale();

    static bool extractCameraData(
        const std::map<int,CameraFrameDataPtr> data,
        std::vector< std::vector<cv::Point3f> >& object_points,
        std::vector< std::vector<cv::Point2f> >& image_points);

    static bool extractStereoData(
        const std::map<int,StereoFrameDataPtr> data,
        std::vector< std::vector<cv::Point3f> >& object_points,
        std::vector< std::vector<cv::Point2f> >& left_points,
        std::vector< std::vector<cv::Point2f> >& right_points);

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

    double mLastTargetScale;

    std::map<int,CameraFrameDataPtr> mLeftCameraData;
    std::map<int,CameraFrameDataPtr> mRightCameraData;
    std::map<int,StereoFrameDataPtr> mStereoData;
};

