#pragma once

#include <osg/Switch>
#include "VisualizationData.h"
#include "VisualizationSettings.h"
#include "ViewerWidgetBase.h"

class ViewerWidget : public ViewerWidgetBase
{
    Q_OBJECT

public:

    ViewerWidget(
        VisualizationDataPort* visudata,
        VisualizationSettingsPort* visusettings,
        QWidget* parent=nullptr);

protected slots:

    void buildScene();
    void applyVisualizationSettings();

protected:

    struct SegmentWrapper
    {
        osg::ref_ptr<osg::Switch> items_switch;
        osg::ref_ptr<osg::Node> mappoints;
        osg::ref_ptr<osg::Node> densepoints;
        osg::ref_ptr<osg::Node> trajectory;
        osg::ref_ptr<osg::Node> rig;
    };

protected:

    void buildSegment(
        FrameList::iterator A,
        FrameList::iterator B );

    static osg::ref_ptr<osg::Node> createRigNode(
        const Sophus::SE3d& leftcamera2world,
        const Sophus::SE3d& rightcamera2world);

    static osg::ref_ptr<osg::Node> createCameraNode();

protected:

    VisualizationDataPort* mVisualizationData;
    VisualizationSettingsPort* mVisualizationSettings;
    osg::ref_ptr<osg::Switch> mSegmentSwitch;
    std::vector<SegmentWrapper> mSegments;
};

