#include <opencv2/imgproc.hpp>
#include <iostream>
#include <QMessageBox>
#include <QPainter>
#include <QPaintEvent>
#include <QWheelEvent>
#include <QMouseEvent>
#include "Tracker.h"
#include "RecordingReader.h"
#include "ManualCameraCalibrationView.h"

ManualCameraCalibrationView::ManualCameraCalibrationView(
    ManualCameraCalibrationParametersPtr params,
    QWidget* parent) : QWidget(parent)
{
    mCurrentFrameId = -1;
    mParams = params;
    mReader.reset(new RecordingReader(mParams->recording, true));

    mReader->open();

    setMinimumSize(320, 200);
}

ManualCameraCalibrationView::~ManualCameraCalibrationView()
{
    mReader->close();
}

void ManualCameraCalibrationView::wheelEvent(QWheelEvent* ev)
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

void ManualCameraCalibrationView::mouseReleaseEvent(QMouseEvent* ev)
{
    mLastMousePosition = ev->pos();

    ev->accept();
}

void ManualCameraCalibrationView::mousePressEvent(QMouseEvent* ev)
{
    mLastMousePosition = ev->pos();

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

    ev->accept();
}

void ManualCameraCalibrationView::removePoint(int id)
{
    std::map<int,FrameData>::iterator it = mFrameData.find(mCurrentFrameId);

    if( it != mFrameData.end() )
    {
        // remove corner.

        it->second.corners.erase(id);

        // remove edges which reference this corner.

        std::vector< std::pair<int,int> >::iterator it2 = it->second.edges.begin();

        while( it2 != it->second.edges.end() )
        {
            if( it2->first == id || it2->second == id )
            {
                *it2 = it->second.edges.back();
                it->second.edges.pop_back();
            }
            else
            {
                it2++;
            }
        }

        // remove frame data is there are no corners left.

        if(it->second.corners.empty())
        {
            mFrameData.erase(it);
        }
    }
}

int ManualCameraCalibrationView::locatePoint(const QPoint& pt)
{
    const double radius = 10.0;

    int ret = -1;

    std::map<int,FrameData>::iterator it = mFrameData.find(mCurrentFrameId);

    if( it != mFrameData.end() )
    {
        double best_dist = 0.0;
        const cv::Point2f winpt(pt.x(), pt.y());
        const cv::Point2f midwin( width()/2, height()/2 );

        for( const std::pair<int,FramePoint>& corner : it->second.corners )
        {
            const cv::Point2f winpt2 = midwin + mZoom.factor*(corner.second.image_point - mZoom.point);

            const cv::Point2f delta = winpt2 - winpt;

            const double dist = delta.x*delta.x + delta.y*delta.y;
            if( dist < radius*radius )
            {
                if(ret < 0 || best_dist > dist)
                {
                    best_dist = dist;
                    ret = corner.first;
                }
            }
        }
    }

    return ret;
}

void ManualCameraCalibrationView::addPoint(const cv::Point2f& pt)
{
    FrameData& fd = mFrameData[mCurrentFrameId];
    
    int i = 0;
    while(fd.corners.find(i) != fd.corners.end())
    {
        i++;
    }
    
    fd.corners[i].image_point = pt;
}

void ManualCameraCalibrationView::mouseMoveEvent(QMouseEvent* ev)
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

void ManualCameraCalibrationView::clear()
{
    std::map<int,FrameData>::iterator it = mFrameData.find(mCurrentFrameId);
    if(it != mFrameData.end())
    {
        mFrameData.erase(it);
        update();
    }
}

void ManualCameraCalibrationView::paintEvent(QPaintEvent* ev)
{
    QPainter p(this);

    p.setBackground(QColor(Qt::black));
    p.eraseRect(rect());

    const int half_width = width()/2;
    const int half_height = height()/2;

    if(mFrame.isNull() == false)
    {
        if(mZoom.valid == false)
        {
            mZoom.init(mFrame);
        }

        cv::Rect source(0,0,mFrame.width(), mFrame.height());

        cv::Rect destination(
            half_width - mZoom.factor*mZoom.point.x,
            half_height - mZoom.factor*mZoom.point.y,
            mFrame.width()*mZoom.factor,
            mFrame.height()*mZoom.factor);

        p.drawImage(
            QRect(destination.x, destination.y, destination.width, destination.height),
            mFrame,
            QRect(source.x, source.y, source.width, source.height));

        std::map<int,FrameData>::iterator it = mFrameData.find(mCurrentFrameId);

        if(it != mFrameData.end())
        {
            FrameData& data = it->second;

            // draw edges.

            p.save();
            p.setPen(QPen(QColor(Qt::green), 2.0));
            for(const std::pair<int,int>& edge : data.edges)
            {
                if( edge.first == edge.second ) throw std::runtime_error("internal error");

                const std::map<int,FramePoint>::iterator pta = data.corners.find(edge.first);
                const std::map<int,FramePoint>::iterator ptb = data.corners.find(edge.second);

                if(pta == data.corners.end() || ptb == data.corners.end()) throw std::runtime_error("internal error");

                const QPointF ptapt(
                    half_width + mZoom.factor*(pta->second.image_point.x - mZoom.point.x),
                    half_height + mZoom.factor*(pta->second.image_point.y - mZoom.point.y) );

                const QPointF ptbpt(
                    half_width + mZoom.factor*(ptb->second.image_point.x - mZoom.point.x),
                    half_height + mZoom.factor*(ptb->second.image_point.y - mZoom.point.y) );

                p.drawLine(ptapt, ptbpt);
            }
            p.restore();

            // draw points.

            p.save();
            p.setPen(QPen(QColor(Qt::black), 2.0));
            for(const std::pair<int,FramePoint>& pt : data.corners)
            {
                if(pt.second.has_object_point)
                {
                    p.setBrush(QColor(Qt::green));
                }
                else
                {
                    p.setBrush(QColor(Qt::white));
                }

                const QPointF pt2(
                    half_width + mZoom.factor*(pt.second.image_point.x - mZoom.point.x),
                    half_height + mZoom.factor*(pt.second.image_point.y - mZoom.point.y) );

                p.drawEllipse(pt2, 7, 7);
            }

            p.restore();
        }
    }

    ev->accept();
}

void ManualCameraCalibrationView::autoDetect()
{
    if( mFrameOpenCV.data )
    {
        target::Tracker tracker;
        tracker.setUnitLength(mParams->scale);

        const bool ret = tracker.track(mFrameOpenCV, false);

        if(ret)
        {
            FrameData& fd = mFrameData[mCurrentFrameId];

            fd.corners.clear();
            fd.edges.clear();

            for(int i=0; i<tracker.objectPoints().size(); i++)
            {
                FramePoint& pt = fd.corners[i];

                pt.has_object_point = true;
                pt.image_point = tracker.imagePoints()[i];
                pt.object_point = tracker.objectPoints()[i];
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
}

bool ManualCameraCalibrationView::getCalibrationData(
    std::vector< std::vector<cv::Point2f> >& image_points,
    std::vector< std::vector<cv::Point3f> >& object_points,
    cv::Size& size)
{
    bool ret = false;

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
                if(fp.second.has_object_point)
                {
                    these_image_points.push_back(fp.second.image_point);
                    these_object_points.push_back(fp.second.object_point);
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

    return ret;
}

void ManualCameraCalibrationView::setMode(Mode mode)
{
    mMode = mode;

    // TODO
}

void ManualCameraCalibrationView::setModeToCorner()
{
    setMode(MODE_CORNER);
}

void ManualCameraCalibrationView::setModeToConnection()
{
    setMode(MODE_CONNECTION);
}

void ManualCameraCalibrationView::setFrame(int frame)
{
    mCurrentFrameId = frame;

    mReader->seek(frame);

    mReader->trigger();

    Image img;
    mReader->read(img);

    if(img.isValid())
    {
        cv::cvtColor(img.getFrame(), mFrameOpenCV, cv::COLOR_BGR2RGB);

        mFrame = QImage(
            mFrameOpenCV.data,
            mFrameOpenCV.cols,
            mFrameOpenCV.rows,
            mFrameOpenCV.step,
            QImage::Format_RGB888);

        //mFrame = img.copy();

        if(mZoom.valid == false)
        {
            mZoom.init(mFrame);
        }

    }
    else
    {
        mFrame = QImage();
        mFrameOpenCV = cv::Mat();
        std::cerr << "Invalid frame!" << std::endl;
    }

    update();
}

void ManualCameraCalibrationView::home()
{
    if(mFrame.isNull())
    {
        mZoom.valid = false;
    }
    else
    {
        mZoom.init(mFrame);
    }

    update();
}

ManualCameraCalibrationView::ZoomData::ZoomData()
{
    valid = false;
    factor = 1.0;
}

void ManualCameraCalibrationView::ZoomData::init(const QImage& img)
{
    valid = true;
    factor = 1.0;
    point.x = img.width() / 2;
    point.y = img.height() / 2;
}

ManualCameraCalibrationView::FramePoint::FramePoint()
{
    has_object_point = false;
}

