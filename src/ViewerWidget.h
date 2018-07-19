#pragma once

#include <memory>
#include <QOpenGLWidget>
#include <osgViewer/Viewer>
#include <osgViewer/GraphicsWindow>

class QMouseEvent;

class ViewerWidget : public QOpenGLWidget
{
    Q_OBJECT

public:

    ViewerWidget(QWidget* parent=nullptr);

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

protected:

    osg::ref_ptr<osgViewer::Viewer> _viewer;
    osg::ref_ptr<osgViewer::GraphicsWindowEmbedded> _window;
    int _updateTimer;
};

