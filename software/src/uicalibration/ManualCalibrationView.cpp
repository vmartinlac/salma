#include <opencv2/imgproc.hpp>
#include <queue>
#include <iostream>
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
    mMode = MODE_LEFT;
    mCurrentFrameId = -1;
    //mSelectedPoint = -1;
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

    /*
    {
        if(ev->button() == Qt::LeftButton)
        {
            const int clicked_point = locatePoint( ev->pos() );

            if(clicked_point >= 0)
            {
                removePoint(clicked_point);

                update();
            }
            else
            {
                const cv::Point2f winpt( ev->pos().x(), ev->pos().y() );

                const cv::Point2f halfwin( width()/2, height()/2 );

                const cv::Point2f framept = mZoom.point + (winpt - halfwin) / mZoom.factor;

                if( 0.0 <= framept.x && framept.x < mFrame.width() && 0.0 <= framept.y && framept.y < mFrame.height() )
                {
                    addPoint(framept);
                    update();
                }
            }
        }
    }
    else if(mMode == MODE_CONNECTION)
    {
        if(mSelectedPoint >= 0)
        {
            const int new_point = locatePoint( ev->pos() );

            if( new_point >= 0 && new_point != mSelectedPoint && mFrameData[mCurrentFrameId].corners.find(mSelectedPoint) != mFrameData[mCurrentFrameId].corners.end() )
            {
                toggleConnection(mSelectedPoint, new_point);
                //mFrameData[mCurrentFrameId].edges.push_back(std::pair<int,int>(mSelectedPoint, new_point));
            }

            mSelectedPoint = -1;

        }
        else
        {
            mSelectedPoint = locatePoint( ev->pos() );
        }

        update();
    }
    */

    mLastMousePosition = ev->pos();
    ev->accept();
}

/*
void ManualCalibrationView::toggleConnection(int corner1, int corner2)
{
    std::map<int,FrameData>::iterator it = mFrameData.find(mCurrentFrameId);

    if(it != mFrameData.end())
    {
        FrameData& fd = it->second;

        std::map<int,FramePoint>::iterator it1 = fd.corners.find(corner1);
        std::map<int,FramePoint>::iterator it2 = fd.corners.find(corner2);

        if(it1 != fd.corners.end() && it2 != fd.corners.end())
        {
            FramePoint& pt1 = it1->second;
            FramePoint& pt2 = it2->second;

            if( std::find(pt1.neighbors.begin(), pt1.neighbors.end(), corner2) == pt1.neighbors.end() )
            {
                // add connection.

                std::array<int,4>::iterator slot1 = std::find(pt1.neighbors.begin(), pt1.neighbors.end(), -1);
                std::array<int,4>::iterator slot2 = std::find(pt2.neighbors.begin(), pt2.neighbors.end(), -1);

                if(slot1 != pt1.neighbors.end() || slot2 != pt2.neighbors.end())
                {
                    if( slot1 == pt1.neighbors.end() || slot2 == pt2.neighbors.end() ) throw std::runtime_error("internal error");

                    *slot1 = corner2;
                    *slot2 = corner1;
                }
            }
            else
            {
                // remove connection.

                std::replace(
                    pt1.neighbors.begin(),
                    pt1.neighbors.end(),
                    corner2,
                    -1);

                std::replace(
                    pt2.neighbors.begin(),
                    pt2.neighbors.end(),
                    corner1,
                    -1);
            }
        }
    }
}
*/

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

void ManualCalibrationView::clear()
{
    /*
    std::map<int,FrameData>::iterator it = mFrameData.find(mCurrentFrameId);
    if(it != mFrameData.end())
    {
        mFrameData.erase(it);
        update();
    }
    */
}

void ManualCalibrationView::paintEvent(QPaintEvent* ev)
{
    QPainter p(this);

    p.setBackground(QColor(Qt::black));
    p.eraseRect(rect());

    const int half_width = width()/2;
    const int half_height = height()/2;

    if(mCurrentImageQt.isNull() == false)
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

        /*
        std::map<int,FrameData>::iterator it = mFrameData.find(mCurrentFrameId);

        if(it != mFrameData.end())
        {
            FrameData& data = it->second;

            // draw edges.

            p.save();
            p.setPen(QPen(QColor(Qt::green), 2.0));
            for(const std::pair<int,FramePoint>& pt : data.corners)
            {
                const QPointF ptapt(
                    half_width + mZoom.factor*(pt.second.image_coords.x - mZoom.point.x),
                    half_height + mZoom.factor*(pt.second.image_coords.y - mZoom.point.y) );

                for(int other : pt.second.neighbors)
                {
                    if(other >= 0 && pt.first < other)
                    {
                        const std::map<int,FramePoint>::iterator ptb = data.corners.find(other);

                        if(ptb == data.corners.end()) throw std::runtime_error("internal error");

                        const QPointF ptbpt(
                            half_width + mZoom.factor*(ptb->second.image_coords.x - mZoom.point.x),
                            half_height + mZoom.factor*(ptb->second.image_coords.y - mZoom.point.y) );

                        p.drawLine(ptapt, ptbpt);
                    }
                }

            }
            p.restore();

            // draw points.

            p.save();
            p.setPen(QPen(QColor(Qt::black), 2.0));
            for(const std::pair<int,FramePoint>& pt : data.corners)
            {
                if(pt.second.has_object_coords)
                {
                    p.setBrush(QColor(Qt::green));
                }
                else
                {
                    p.setBrush(QColor(Qt::white));
                }

                const QPointF pt2(
                    half_width + mZoom.factor*(pt.second.image_coords.x - mZoom.point.x),
                    half_height + mZoom.factor*(pt.second.image_coords.y - mZoom.point.y) );

                p.drawEllipse(pt2, 7, 7);
            }

            p.restore();
        }
        */
    }

    ev->accept();
}

void ManualCalibrationView::autoDetect()
{
    /*
    if( mFrameOpenCV.data )
    {
        target::Tracker tracker;
        tracker.setUnitLength(1.0); // TODO: ask the user by input dialog.

        const bool ret = tracker.track(mFrameOpenCV, false);

        if(ret)
        {
            FrameData& fd = mFrameData[mCurrentFrameId];

            fd.corners.clear();

            for(int i=0; i<tracker.objectPoints().size(); i++)
            {
                FramePoint& pt = fd.corners[i];

                pt.has_object_coords = true;
                pt.image_coords = tracker.imagePoints()[i];
                pt.object_coords = tracker.integerObjectCoords()[i];
            }

            update();
        }
        else
        {
            //mFrameData.erase(mCurrentFrameId);

            //update();

            QMessageBox::critical(this, "AutoDetect failed", "AutoDetect failed!");
        }

    }
    */
}

bool ManualCalibrationView::getCalibrationData(
    std::vector< std::vector<cv::Point2f> >& image_points,
    std::vector< std::vector<cv::Point3f> >& object_points,
    cv::Size& size)
{
    bool ret = false;
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
    return ret;
}

void ManualCalibrationView::setMode(Mode mode)
{
    mMode = mode;
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

