#include <iostream>

#include <osg/Geometry>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osg/Point>
#include <osg/PrimitiveSet>
#include <osg/StateSet>
#include <osg/Material>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osgGA/TrackballManipulator>

#include <QEvent>
#include <QTimerEvent>
#include <QMouseEvent>

#include "ViewerWidget.h"

ViewerWidget::ViewerWidget(SLAMOutput* slam, QWidget* parent) :
    m_slam(slam),
    QOpenGLWidget(parent)
{
    osg::ref_ptr<osg::Group> data = new osg::Group();

    // create landmarks osg representation.

    {
        _landmarks = new osg::Vec3Array();
        //_landmarks->setDataVariance(osg::Object::DYNAMIC);
        _landmarks->push_back( osg::Vec3(-3, -3, -3) );
        _landmarks->push_back( osg::Vec3(3, 3, -3) );

        osg::ref_ptr<osg::Vec3Array> lm_colors = new osg::Vec3Array();
        lm_colors->push_back( osg::Vec3(0.0, 1.0, 0.0) );

        _draw_landmarks = new osg::DrawArrays( osg::PrimitiveSet::POINTS, 0, 2 );
        //_draw_landmarks->setDataVariance(osg::Object::DYNAMIC);

        _landmarks_geometry = new osg::Geometry();
        _landmarks_geometry->setUseDisplayList(false);
        _landmarks_geometry->addPrimitiveSet(_draw_landmarks);
        _landmarks_geometry->setVertexArray(_landmarks);
        _landmarks_geometry->setColorArray(lm_colors, osg::Array::BIND_OVERALL);
        _landmarks_geometry->getOrCreateStateSet()->setAttribute(new osg::Point(10.0), osg::StateAttribute::ON);

        osg::ref_ptr<osg::Geode> lm_geode = new osg::Geode();
        lm_geode->addDrawable(_landmarks_geometry);

        data->addChild(lm_geode);
    }

    // create camera osg representation.

    {
        const double l = 1.0; // TODO: derive those constants from user data. Do not use arbitrary units not derived from what was given by the user.
        const double kx = 0.6;
        const double ky = 0.3;

        osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();
        vertices->push_back( osg::Vec3(0.0, 0.0, 0.0) );
        vertices->push_back( osg::Vec3(-kx*l, -ky*l, l) );
        vertices->push_back( osg::Vec3(kx*l, -ky*l, l) );
        vertices->push_back( osg::Vec3(kx*l, ky*l, l) );
        vertices->push_back( osg::Vec3(-kx*l, ky*l, l) );

        osg::ref_ptr<osg::Vec3Array> colors = new osg::Vec3Array();
        colors->push_back( osg::Vec3(1.0, 0.0, 0.0) );

        osg::ref_ptr<osg::DrawElementsUInt> draw_elements = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES);
        draw_elements->addElement(0);
        draw_elements->addElement(1);
        draw_elements->addElement(0);
        draw_elements->addElement(2);
        draw_elements->addElement(0);
        draw_elements->addElement(3);
        draw_elements->addElement(0);
        draw_elements->addElement(4);
        draw_elements->addElement(1);
        draw_elements->addElement(2);
        draw_elements->addElement(2);
        draw_elements->addElement(3);
        draw_elements->addElement(3);
        draw_elements->addElement(4);
        draw_elements->addElement(4);
        draw_elements->addElement(1);

        osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
        geom->setVertexArray(vertices);
        geom->setColorArray(colors, osg::Array::BIND_OVERALL);
        geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, 1));
        geom->addPrimitiveSet(draw_elements);
        geom->getOrCreateStateSet()->setAttribute(new osg::Point(8.0), osg::StateAttribute::ON);
        //geom->getOrCreateStateSet()->setAttribute(new osg::LineWidth(2.0), osg::StateAttribute::ON);

        osg::ref_ptr<osg::Geode> geode = new osg::Geode();
        geode->addDrawable(geom);

        _camera = new osg::PositionAttitudeTransform();
        _camera->addChild(geode);

        data->addChild(_camera);
    }

    osgGA::TrackballManipulator* manipulator = new osgGA::TrackballManipulator;

    _viewer = new osgViewer::Viewer;
    _viewer->setCameraManipulator(manipulator);
    _viewer->setSceneData(data);
    _viewer->setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
    _viewer->setRunFrameScheme(osgViewer::Viewer::ON_DEMAND);

    _window = _viewer->setUpViewerAsEmbeddedInWindow(0, 0, width(), height());

    //setMouseTracking(true);
    setFocusPolicy(Qt::StrongFocus);
    setMinimumSize(100, 100);
    _updateTimer = startTimer(30);

    connect(slam, SIGNAL(updated()), this, SLOT(refresh()));

    _viewer->home();
}

void ViewerWidget::timerEvent(QTimerEvent* ev)
{
    if(ev->timerId() == _updateTimer)
    {
        update();
    }
}

void ViewerWidget::paintGL()
{
    _viewer->frame();
}

void ViewerWidget::mouseMoveEvent(QMouseEvent* event)
{
    _window->getEventQueue()->mouseMotion(event->x(), event->y());
}

void ViewerWidget::initializeGL()
{
   ;
}

void ViewerWidget::mousePressEvent(QMouseEvent* event)
{
    unsigned int button = 0;
    switch (event->button()){
    case Qt::LeftButton:
        button = 1;
        break;
    case Qt::MiddleButton:
        button = 2;
        break;
    case Qt::RightButton:
        button = 3;
        break;
    default:
        break;
    }
    _window->getEventQueue()->mouseButtonPress(event->x(), event->y(), button);
}

void ViewerWidget::mouseReleaseEvent(QMouseEvent* event)
{
    unsigned int button = 0;

    switch (event->button())
    {
    case Qt::LeftButton:
        button = 1;
        break;
    case Qt::MiddleButton:
        button = 2;
        break;
    case Qt::RightButton:
        button = 3;
        break;
    default:
        break;
    }

    _window->getEventQueue()->mouseButtonRelease(event->x(), event->y(), button);
}

void ViewerWidget::wheelEvent(QWheelEvent* event)
{
    const int delta = event->delta();
    osgGA::GUIEventAdapter::ScrollingMotion motion = delta > 0 ?  osgGA::GUIEventAdapter::SCROLL_UP : osgGA::GUIEventAdapter::SCROLL_DOWN;
    _window->getEventQueue()->mouseScroll(motion);
}

bool ViewerWidget::event(QEvent* event)
{
    bool handled = QOpenGLWidget::event(event);

    switch( event->type() )
    {
    case QEvent::KeyPress:
    case QEvent::KeyRelease:
    case QEvent::MouseButtonDblClick:
    case QEvent::MouseButtonPress:
    case QEvent::MouseButtonRelease:
    case QEvent::MouseMove:
        update();
        break;
    /*
    case QEvent::Timer:
        update();
        break;
    */
    default:
        break;
    };

    return handled;
}

void ViewerWidget::resizeGL(int width, int height)
{
    //_window->getEventQueue()->windowResize( this->x(), this->y(), width, height );
    _window->resized( this->x(), this->y(), width, height );

   /*
   std::vector<osg::Camera*> cameras;
   _viewer->getCameras( cameras );

   assert( cameras.size() == 1 );

   cameras.front()->setViewport( 0, 0, this->width(), this->height() );
   */
}

void ViewerWidget::refresh()
{
    m_slam->beginRead();

    _camera->setPosition( osg::Vec3(
        m_slam->position.x(),
        m_slam->position.y(),
        m_slam->position.z() ));

    _camera->setAttitude( osg::Quat(
        m_slam->attitude.x(),
        m_slam->attitude.y(),
        m_slam->attitude.z(),
        m_slam->attitude.w() ));

    _landmarks->clear();
    _landmarks->reserve( m_slam->landmarks.size() );
    for( SLAMOutputLandmark& lm : m_slam->landmarks )
    {
        _landmarks->push_back( osg::Vec3(
            lm.position.x(),
            lm.position.y(),
            lm.position.z() ));
    }

    _landmarks_geometry->dirtyBound();

    _draw_landmarks->set(osg::PrimitiveSet::POINTS, 0, _landmarks->size());

    m_slam->endRead();
}

void ViewerWidget::home()
{
    _viewer->home();
}

