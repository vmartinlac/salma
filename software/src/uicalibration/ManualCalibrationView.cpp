#include <opencv2/imgproc.hpp>
#include <queue>
#include <iostream>
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

                for(cv::Point2f pt : it->second->image_points)
                {
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

                for( std::pair<cv::Point2f,cv::Point2f> pair : it->second->correspondances)
                {
                    const QPointF pt_left(
                        half_width + mZoom.factor * (mLeftROI.x + pair.first.x - mZoom.point.x),
                        half_height + mZoom.factor * (mLeftROI.y + pair.first.y - mZoom.point.y) );

                    const QPointF pt_right(
                        half_width + mZoom.factor * (mRightROI.x + pair.second.x - mZoom.point.x),
                        half_height + mZoom.factor * (mRightROI.y + pair.second.y - mZoom.point.y) );

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
                const double scale = QInputDialog::getDouble(this, "Target scale", "Size of a square on the target?", mLastTargetScale, 0.0, 100000.0, 5, &ok);

                if(ok)
                {
                    mLastTargetScale = scale;
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

                data->object_points = tracker.objectPoints();
                data->image_points = tracker.imagePoints();

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
            std::vector< std::pair<cv::Point2f,cv::Point2f> > correspondances;
            bool ok = true;

            if(ok)
            {
                const double scale = QInputDialog::getDouble(this, "Target scale", "Size of a square on the target?", mLastTargetScale, 0.0, 100000.0, 5, &ok);

                if(ok)
                {
                    mLastTargetScale = scale;
                    left_tracker.setUnitLength(scale);
                    right_tracker.setUnitLength(scale);
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
                        correspondances.push_back(std::pair<cv::Point2f,cv::Point2f>(
                            left_tracker.imagePoints()[i],
                            right_tracker.imagePoints()[j]
                        ));
                    }
                }

                ok = (correspondances.size() >= 8);
            }

            if(ok)
            {
                StereoFrameDataPtr data(new StereoFrameData());

                data->correspondances.swap(correspondances);

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

bool ManualCalibrationView::doCalibrate(StereoRigCalibrationPtr& calib)
{
    bool ret = false;

    //
    calib.reset(new StereoRigCalibration());
    calib->id = -1;
    calib->name = mParams->name.toStdString();
    calib->date.clear();

    calib->cameras[0].calibration_matrix = cv::Mat::zeros(3,3,CV_64F);
    calib->cameras[0].calibration_matrix.at<double>(0,0) = 1070.0;
    calib->cameras[0].calibration_matrix.at<double>(1,1) = 1070.0;
    calib->cameras[0].calibration_matrix.at<double>(0,2) = 512.0;
    calib->cameras[0].calibration_matrix.at<double>(1,2) = 384.0;
    calib->cameras[0].image_size = cv::Size(1024, 768);
    calib->cameras[0].photometric_lut = cv::Mat(1, 256, CV_32FC3);

    calib->cameras[1] = calib->cameras[0];
    calib->cameras[1].camera_to_rig.translation() << 0.0, 10.0, 0.0;

    return true;
    //
    /*

    if( mFrameOpenCV.data )
    {
        size = mFrameOpenCV.size();

        image_points.clear();
        object_points.clear();

        for(const std::pair<int,FrameData>& fd : mFrameData)
        {
            std::vector<cv::Point2f> these_image_points;
            std::vector<cv::Point3f> these_object_points;

            for(const std::pair<int,FramePoint>& fp : fd.second.corners)
            {
                if(fp.second.has_object_coords)
                {
                    cv::Point3f object_point(
                        double(fp.second.object_coords.x), // TODO
                        double(fp.second.object_coords.y), // TODO
                        0.0);

                    these_image_points.push_back(fp.second.image_coords);
                    these_object_points.push_back(object_point);
                }
            }

            if(these_image_points.size() != these_object_points.size()) throw std::runtime_error("internal error");

            if(these_image_points.empty() == false)
            {
                image_points.emplace_back( std::move(these_image_points) );
                object_points.emplace_back( std::move(these_object_points) );
            }
        }

        ret = true;
    }
*/

    /*
    CameraCalibrationPtr calib(new CameraCalibration());
    bool ok = true;
    const char* err = "";
    double projection_err = 0.0;

    std::vector< std::vector<cv::Point2f> > image_points;
    std::vector< std::vector<cv::Point3f> > object_points;

    // Retrieve calibration data.

    if(ok)
    {
        calib->name = mParameters->name.toStdString();

        ok = mView->getCalibrationData(
            image_points,
            object_points,
            calib->image_size);

        err = "Internal error";
    }

    // Temove views on which there are not enough points.
    // Then check that we have enough points for the calibration.

    if(ok)
    {
        if(image_points.size() != object_points.size()) throw std::runtime_error("internal error");

        {
            int i = 0;

            while(i<image_points.size())
            {
                if(image_points[i].size() != object_points[i].size()) throw std::runtime_error("internal error");

                if(image_points[i].size() >= 3*3)
                {
                    i++;
                }
                else
                {
                    image_points[i] = std::move(image_points.back());
                    object_points[i] = std::move(object_points.back());

                    image_points.pop_back();
                    object_points.pop_back();
                }
            }
        }

        ok = (image_points.size() >= 3);
        err = "Not enough points or orientations of the target!";
    }

    // Call OpenCV for the calibration.

    if(ok)
    {
        cv::Mat rvecs;
        cv::Mat tvecs;

        projection_err = cv::calibrateCamera(
            object_points,
            image_points,
            calib->image_size, // TODO: set this beforehand!
            calib->calibration_matrix,
            calib->distortion_coefficients,
            rvecs,
            tvecs);
    }

    // Save calibration into the project.

    if(ok)
    {
        int camera_id;
        ok = mProject->saveCamera(calib, camera_id);
        err = "Could not save camera!";
    }

    // Tell outcome to the user.

    if(ok)
    {
        QMessageBox::information(this, "Success", "Successful calibration! Reprojection error is " + QString::number(projection_err));
        QDialog::accept();
    }
    else
    {
        QMessageBox::critical(this, "Error", err);
    }
    */

    return ret;
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

