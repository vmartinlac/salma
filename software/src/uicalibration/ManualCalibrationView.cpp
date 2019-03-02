#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <queue>
#include <iostream>
#include <QJsonDocument>
#include <QJsonObject>
#include <QByteArray>
#include <QInputDialog>
#include <QMessageBox>
#include <QPainter>
#include <QPaintEvent>
#include <QWheelEvent>
#include <QMouseEvent>
#include "Tracker.h"
#include "RecordingReader.h"
#include "ManualCalibrationView.h"

ManualCalibrationView::ManualCalibrationView(
    ManualCalibrationParametersPtr params,
    QWidget* parent) : QWidget(parent)
{
    mLastTargetScale = 1.0;
    mMode = MODE_LEFT;
    mCurrentFrameId = -1;
    mParams = params;
    mReader.reset(new RecordingReader(mParams->recording, true));

    mReader->open();

    setMinimumSize(320, 200);
}

ManualCalibrationView::~ManualCalibrationView()
{
    mReader->close();
}

void ManualCalibrationView::wheelEvent(QWheelEvent* ev)
{
    const double factor_max = 20.0;
    const double factor_min = 1.0/20.0;
    const double speed = 2.0e-3;

    if(mZoom.valid)
    {
        const QPoint delta = ev->angleDelta();

        if(delta.isNull() == false)
        {
            const double new_factor = mZoom.factor * std::exp(speed * delta.y());
            mZoom.factor = std::max(factor_min, std::min(factor_max, new_factor));
            update();
        }
    }

    mLastMousePosition = ev->pos();

    ev->accept();
}

void ManualCalibrationView::mouseReleaseEvent(QMouseEvent* ev)
{
    mLastMousePosition = ev->pos();
    ev->accept();
}

void ManualCalibrationView::mousePressEvent(QMouseEvent* ev)
{
    mLastMousePosition = ev->pos();
    ev->accept();
}

void ManualCalibrationView::mouseMoveEvent(QMouseEvent* ev)
{
    if(ev->buttons() & Qt::RightButton)
    {
        if(mZoom.valid)
        {
            const QPoint delta = ev->pos() - mLastMousePosition;
            mZoom.point.x -= delta.x() / mZoom.factor;
            mZoom.point.y -= delta.y() / mZoom.factor;
            update();
        }
    }

    mLastMousePosition = ev->pos();

    ev->accept();
}

void ManualCalibrationView::doClear()
{
    if(0 <= mCurrentFrameId && mCurrentFrameId < mParams->recording->num_frames)
    {
        if( mMode == MODE_LEFT )
        {
            mLeftCameraData.erase(mCurrentFrameId);
        }
        else if(mMode == MODE_RIGHT )
        {
            mRightCameraData.erase(mCurrentFrameId);
        }
        else if(mMode == MODE_STEREO)
        {
            mStereoData.erase(mCurrentFrameId);
        }
        else if(mMode == MODE_PHOTOMETRIC)
        {
            mPhotometricData.erase(mCurrentFrameId);
        }
        else
        {
            throw std::runtime_error("internal error");
        }

        update();
        listOfFramesWithDataChanged();
    }
}

void ManualCalibrationView::paintEvent(QPaintEvent* ev)
{
    QPainter p(this);

    p.setBackground(QColor(Qt::black));
    p.eraseRect(rect());

    const int half_width = width()/2;
    const int half_height = height()/2;

    if(mCurrentImageQt.isNull() == false && 0 <= mCurrentFrameId && mCurrentFrameId < mParams->recording->num_frames)
    {
        if(mZoom.valid == false)
        {
            mZoom.init(this, mCurrentImageQt);
        }

        cv::Rect source(0,0,mCurrentImageQt.width(), mCurrentImageQt.height());

        cv::Rect destination(
            half_width - mZoom.factor*mZoom.point.x,
            half_height - mZoom.factor*mZoom.point.y,
            mCurrentImageQt.width()*mZoom.factor,
            mCurrentImageQt.height()*mZoom.factor);

        p.drawImage(
            QRect(destination.x, destination.y, destination.width, destination.height),
            mCurrentImageQt,
            QRect(source.x, source.y, source.width, source.height));

        if(mMode == MODE_LEFT || mMode == MODE_RIGHT)
        {
            cv::Rect ROI;
            std::map<int,CameraFrameDataPtr>* frame_data = nullptr;

            if(mMode == MODE_LEFT)
            {
                ROI = mLeftROI;
                frame_data = &mLeftCameraData;
            }
            else if(mMode == MODE_RIGHT)
            {
                ROI = mRightROI;
                frame_data = &mRightCameraData;
            }
            else
            {
                throw std::logic_error("internal error");
            }

            std::map<int,CameraFrameDataPtr>::iterator it = frame_data->find(mCurrentFrameId);

            if(it != frame_data->end())
            {
                p.save();
                p.setPen(QPen(QColor(Qt::black), 2.0));
                p.setBrush(QColor(Qt::white));

                for(MonoCorrespondance& corr : it->second->points)
                {
                    const cv::Point2f pt = corr.image_point;

                    const QPointF pt2(
                        half_width + mZoom.factor * (ROI.x + pt.x - mZoom.point.x),
                        half_height + mZoom.factor * (ROI.y + pt.y - mZoom.point.y) );

                    p.drawEllipse(pt2, 7, 7);
                }

                p.restore();
            }
        }
        else if(mMode == MODE_STEREO)
        {
            std::map<int,StereoFrameDataPtr>::iterator it = mStereoData.find(mCurrentFrameId);

            if(it != mStereoData.end())
            {
                p.save();
                p.setPen(QPen(QColor(Qt::black), 2.0));
                p.setBrush(QColor(Qt::white));

                for( StereoCorrespondance& corr : it->second->points)
                {
                    const QPointF pt_left(
                        half_width + mZoom.factor * (mLeftROI.x + corr.left_image_point.x - mZoom.point.x),
                        half_height + mZoom.factor * (mLeftROI.y + corr.left_image_point.y - mZoom.point.y) );

                    const QPointF pt_right(
                        half_width + mZoom.factor * (mRightROI.x + corr.right_image_point.x - mZoom.point.x),
                        half_height + mZoom.factor * (mRightROI.y + corr.right_image_point.y - mZoom.point.y) );

                    p.drawEllipse(pt_left, 7, 7);
                    p.drawEllipse(pt_right, 7, 7);
                }

                p.restore();
            }
        }
        else if(mMode == MODE_PHOTOMETRIC)
        {
        }
        else
        {
            throw std::runtime_error("internal error");
        }
    }

    ev->accept();
}

void ManualCalibrationView::doTake()
{
    if(0 <= mCurrentFrameId && mCurrentFrameId < mParams->recording->num_frames)
    {
        if( mMode == MODE_LEFT || mMode == MODE_RIGHT )
        {
            target::Tracker tracker;
            bool ok = true;

            if(ok)
            {
                double scale = 1.0;

                ok = askScale(scale);

                if(ok)
                {
                    tracker.setUnitLength(scale);
                }
            }

            if(ok)
            {
                if( mMode == MODE_LEFT )
                {
                    ok = tracker.track(mCurrentImage.getFrame(0), false);
                }
                else if( mMode == MODE_RIGHT)
                {
                    ok = tracker.track(mCurrentImage.getFrame(1), false);
                }
                else
                {
                    throw std::logic_error("internal error");
                }
            }

            if(ok)
            {
                ok = ( tracker.imagePoints().size() >= 15 );
            }

            if(ok)
            {
                CameraFrameDataPtr data(new CameraFrameData());

                const int N = tracker.objectPoints().size();
                for(int i=0; i<N; i++)
                {
                    data->points.emplace_back();
                    MonoCorrespondance& corr = data->points.back();

                    corr.image_point = tracker.imagePoints()[i];
                    corr.object_point = tracker.objectPoints()[i];
                }

                if( mMode == MODE_LEFT )
                {
                    mLeftCameraData[mCurrentFrameId] = data;
                }
                else if( mMode == MODE_RIGHT )
                {
                    mRightCameraData[mCurrentFrameId] = data;
                }
                else
                {
                    throw std::logic_error("internal error");
                }

                update();
                listOfFramesWithDataChanged();
            }
        }
        else if(mMode == MODE_STEREO)
        {
            target::Tracker left_tracker;
            target::Tracker right_tracker;
            std::vector<StereoCorrespondance> correspondances;
            bool ok = true;

            if(ok)
            {
                double target_scale = 1.0;

                ok = askScale(target_scale);

                if(ok)
                {
                    left_tracker.setUnitLength(target_scale);
                    right_tracker.setUnitLength(target_scale);
                }
            }

            if(ok)
            {
                ok =
                    left_tracker.track(mCurrentImage.getFrame(0), true) &&
                    right_tracker.track(mCurrentImage.getFrame(1), true); 
            }

            if(ok)
            {
                const int N_left = left_tracker.imagePoints().size();
                const int N_right = right_tracker.imagePoints().size();

                for(int i=0; i<N_left; i++)
                {
                    int j = 0;
                    while( j<N_right && right_tracker.pointIds()[j] != left_tracker.pointIds()[i] )
                    {
                        j++;
                    }

                    if(j < N_right)
                    {
                        correspondances.emplace_back();
                        StereoCorrespondance& corr = correspondances.back();
                        corr.left_image_point = left_tracker.imagePoints()[i];
                        corr.right_image_point = right_tracker.imagePoints()[j];
                        corr.object_point = left_tracker.objectPoints()[i];
                    }
                }

                ok = (correspondances.size() >= 8);
            }

            if(ok)
            {
                StereoFrameDataPtr data(new StereoFrameData());

                data->points.swap(correspondances);

                mStereoData[mCurrentFrameId] = data;

                update();
                listOfFramesWithDataChanged();
            }
        }
        else if(mMode == MODE_PHOTOMETRIC)
        {
        }
        else
        {
            throw std::runtime_error("internal error");
        }
    }
}

bool ManualCalibrationView::extractCameraData(
    const std::map<int,CameraFrameDataPtr> data,
    std::vector< std::vector<cv::Point3f> >& object_points,
    std::vector< std::vector<cv::Point2f> >& image_points)
{
    const int min_num_points_per_view = 10;
    const int min_num_views = 4;

    image_points.clear();
    object_points.clear();

    for(std::pair<int,CameraFrameDataPtr> item : data)
    {
        if( item.second->points.size() >= min_num_points_per_view )
        {
            object_points.emplace_back();
            image_points.emplace_back();

            std::transform(
                item.second->points.begin(),
                item.second->points.end(),
                std::back_inserter(image_points.back()),
                [] ( const MonoCorrespondance& corr ) { return corr.image_point; });

            std::transform(
                item.second->points.begin(),
                item.second->points.end(),
                std::back_inserter(object_points.back()),
                [] ( const MonoCorrespondance& corr ) { return corr.object_point; });

            if( object_points.back().size() != image_points.back().size() ) throw std::logic_error("internal error");
        }
    }

    if( object_points.size() != image_points.size() ) throw std::logic_error("internal error");

    if( image_points.size() >= min_num_views )
    {
        return true;
    }
    else
    {
        image_points.clear();
        object_points.clear();

        return false;
    }
}

bool ManualCalibrationView::extractStereoData(
    std::map<int,StereoFrameDataPtr> data,
    std::vector< std::vector<cv::Point3f> >& object_points,
    std::vector< std::vector<cv::Point2f> >& left_points,
    std::vector< std::vector<cv::Point2f> >& right_points)
{
    const int min_num_points_per_view = 8;
    const int min_num_views = 4;

    object_points.clear();
    left_points.clear();
    right_points.clear();

    for(std::pair<int,StereoFrameDataPtr> item : data )
    {
        if( item.second->points.size() >= min_num_points_per_view )
        {
            object_points.emplace_back();
            left_points.emplace_back();
            right_points.emplace_back();

            std::transform(
                item.second->points.begin(),
                item.second->points.end(),
                std::back_inserter(object_points.back()),
                [] ( const StereoCorrespondance& corr ) { return corr.object_point; });

            std::transform(
                item.second->points.begin(),
                item.second->points.end(),
                std::back_inserter(left_points.back()),
                [] ( const StereoCorrespondance& corr ) { return corr.left_image_point; });

            std::transform(
                item.second->points.begin(),
                item.second->points.end(),
                std::back_inserter(right_points.back()),
                [] ( const StereoCorrespondance& corr ) { return corr.right_image_point; });

            if( object_points.back().size() != left_points.back().size() || object_points.back().size() != right_points.back().size() ) throw std::logic_error("internal error");
        }
    }

    if( object_points.size() != left_points.size() || object_points.size() != right_points.size() ) throw std::logic_error("internal error");

    if( object_points.size() >= min_num_views )
    {
        return true;
    }
    else
    {
        object_points.clear();
        left_points.clear();
        right_points.clear();

        return false;
    }
}

bool ManualCalibrationView::doCalibrate(StereoRigCalibrationPtr& calib)
{
    calib.reset(new StereoRigCalibration());
    bool ok = true;

    calib->id = -1;
    calib->name = mParams->name.toStdString();
    calib->date.clear();

    std::vector< std::vector<cv::Point3f> > left_object_points;
    std::vector< std::vector<cv::Point2f> > left_image_points;

    std::vector< std::vector<cv::Point3f> > right_object_points;
    std::vector< std::vector<cv::Point2f> > right_image_points;

    std::vector< std::vector<cv::Point3f> > stereo_object_points;
    std::vector< std::vector<cv::Point2f> > stereo_left_points;
    std::vector< std::vector<cv::Point2f> > stereo_right_points;

    if(ok)
    {
        ok = mCurrentImage.isValid();
    }

    if(ok)
    {
        calib->cameras[0].image_size = mCurrentImage.getFrame(0).size();
        calib->cameras[1].image_size = mCurrentImage.getFrame(1).size();
        calib->cameras[0].photometric_lut = cv::Mat::zeros(1, 256, CV_32FC3);
        calib->cameras[1].photometric_lut = cv::Mat::zeros(1, 256, CV_32FC3);
    }

    if(ok)
    {
        ok = extractCameraData(mLeftCameraData, left_object_points, left_image_points);
    }

    if(ok)
    {
        ok = extractCameraData(mRightCameraData, right_object_points, right_image_points);
    }

    if(ok)
    {
        ok = extractStereoData(mStereoData, stereo_object_points, stereo_left_points, stereo_right_points);
    }

    if(ok)
    {
        cv::Mat rvec;
        cv::Mat tvec;

        const double err = cv::calibrateCamera(
            left_object_points,
            left_image_points,
            calib->cameras[0].image_size,
            calib->cameras[0].calibration_matrix,
            calib->cameras[0].distortion_coefficients,
            rvec,
            tvec);

        std::cout << "left calibration error = " << err << std::endl;
    }

    if(ok)
    {
        cv::Mat rvec;
        cv::Mat tvec;

        const double err = cv::calibrateCamera(
            right_object_points,
            right_image_points,
            calib->cameras[1].image_size,
            calib->cameras[1].calibration_matrix,
            calib->cameras[1].distortion_coefficients,
            rvec,
            tvec);

        std::cout << "right calibration error = " << err << std::endl;
    }

    if(ok)
    {
        cv::Mat mat_R;
        cv::Mat mat_T;
        cv::Mat mat_E;
        cv::Mat mat_F;

        Eigen::Matrix3d Rbis;
        Eigen::Vector3d Tbis;

        const double err = cv::stereoCalibrate(
            stereo_object_points,
            stereo_left_points,
            stereo_right_points,
            calib->cameras[0].calibration_matrix,
            calib->cameras[0].distortion_coefficients,
            calib->cameras[1].calibration_matrix,
            calib->cameras[1].distortion_coefficients,
            calib->cameras[0].image_size,
            mat_R,
            mat_T,
            mat_E,
            mat_F,
            cv::CALIB_FIX_INTRINSIC);

        cv::cv2eigen<double,3,3>(mat_R, Rbis);
        cv::cv2eigen<double,3,1>(mat_T, Tbis);

        calib->cameras[0].camera_to_rig = Sophus::SE3d();
        calib->cameras[1].camera_to_rig.setRotationMatrix(Rbis.transpose());
        calib->cameras[1].camera_to_rig.translation() = -Rbis.transpose() * Tbis;

        std::cout << "stereo calibration error = " << err << std::endl;

        // TODO compare OpenCV and own essential and fundamental matrices.
    }

    if(ok)
    {
        // TODO: photometric calibration.
    }

    if(ok)
    {
        QJsonDocument doc(calib->toJson().toObject());

        std::cout << doc.toJson().data() << std::endl;
    }
    else
    {
        calib.reset();
    }

    return ok;
}

void ManualCalibrationView::setMode(Mode mode)
{
    mMode = mode;

    update();
    listOfFramesWithDataChanged();
}

void ManualCalibrationView::setFrame(int frame)
{
    mCurrentFrameId = frame;

    mReader->seek(frame);

    mReader->trigger();

    mReader->read(mCurrentImage);

    if(mCurrentImage.isValid())
    {
        cv::Mat left = mCurrentImage.getFrame(0);
        cv::Mat right = mCurrentImage.getFrame(1);

        mLeftROI = cv::Rect(0, 0, left.cols, left.rows);
        mRightROI = cv::Rect(left.cols, 0, right.cols, right.rows);

        mCurrentImageQt = QImage( left.cols + right.cols, std::max(left.rows, right.rows), QImage::Format_RGB888 );
        //mCurrentImageQt.fill(Qt::black);

        cv::Mat wrapper(
            mCurrentImageQt.height(),
            mCurrentImageQt.width(),
            CV_8UC3,
            mCurrentImageQt.bits(),
            mCurrentImageQt.bytesPerLine());

        cv::cvtColor( left, wrapper(mLeftROI), cv::COLOR_BGR2RGB );
        cv::cvtColor( right, wrapper(mRightROI), cv::COLOR_BGR2RGB );

        if(mZoom.valid == false)
        {
            mZoom.init(this, mCurrentImageQt);
        }

    }
    else
    {
        mCurrentImageQt = QImage();
        std::cerr << "Invalid frame!" << std::endl;
    }

    update();
}

void ManualCalibrationView::home()
{
    if( mCurrentImageQt.isNull() )
    {
        mZoom.valid = false;
    }
    else
    {
        mZoom.init(this, mCurrentImageQt);
    }

    update();
}

ManualCalibrationView::ZoomData::ZoomData()
{
    valid = false;
    factor = 1.0;
}

void ManualCalibrationView::ZoomData::init(QWidget* win, const QImage& img)
{
    valid = true;
    factor = double(4 * win->width()) / double(5 * img.width());
    point.x = img.width() / 2;
    point.y = img.height() / 2;
}

void ManualCalibrationView::enumerateFramesWithData(std::vector<int>& list)
{
    list.clear();

    if( mMode == MODE_LEFT )
    {
        for(std::pair<int,CameraFrameDataPtr> item : mLeftCameraData)
        {
            list.push_back(item.first);
        }
    }
    else if(mMode == MODE_RIGHT )
    {
        for(std::pair<int,CameraFrameDataPtr> item : mRightCameraData)
        {
            list.push_back(item.first);
        }
    }
    else if(mMode == MODE_STEREO)
    {
        for(std::pair<int,StereoFrameDataPtr> item : mStereoData)
        {
            list.push_back(item.first);
        }
    }
    else if(mMode == MODE_PHOTOMETRIC)
    {
        for(std::pair<int,PhotometricFrameDataPtr> item : mPhotometricData)
        {
            list.push_back(item.first);
        }
    }
    else
    {
        throw std::runtime_error("internal error");
    }
}

bool ManualCalibrationView::askScale(double& scale)
{
    bool ok = true;

    scale = QInputDialog::getDouble(this, "Target scale", "Size of a square on the target?", mLastTargetScale, 0.0, 100000.0, 5, &ok);

    if(ok)
    {
        mLastTargetScale = scale;
    }

    return ok;
}

