#pragma once

#include <memory>
#include <QOpenGLWidget>
#include <osgViewer/Viewer>
#include <osg/PositionAttitudeTransform>
#include <osg/Geometry>
#include <osg/PrimitiveSet>
#include <osg/Array>
#include <osgViewer/GraphicsWindow>

class QMouseEvent;

class ViewerWidgetBase : public QOpenGLWidget
{
    Q_OBJECT

public:

    ViewerWidgetBase(QWidget* parent=nullptr);

protected:

    void paintGL() override;
    void resizeGL(int width, int height) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    bool event(QEvent* event) override;
    void timerEvent(QTimerEvent* event) override;
    void initializeGL() override;

public slots:

    void home();

protected slots:

    void refresh();

protected:

    osg::ref_ptr<osgViewer::Viewer> _viewer;
    osg::ref_ptr<osgViewer::GraphicsWindowEmbedded> _window;
    osg::ref_ptr<osg::PositionAttitudeTransform> _camera;
    osg::ref_ptr<osg::Vec3Array> _landmarks;
    osg::ref_ptr<osg::DrawArrays> _draw_landmarks;
    osg::ref_ptr<osg::Geometry> _landmarks_geometry;
    int _updateTimer;
};

