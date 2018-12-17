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

    enum VisualizationItem
    {
        ITEM_RIG=0,
        ITEM_DENSEPOINTS=1,
        ITEM_MAPPOINTS=2,
        ITEM_TRAJECTORY=3
    };

    struct SegmentData
    {
        osg::ref_ptr<osg::Switch> items;
        std::map<int, int> map;
    };

protected:

    void addSegment(
        ReconstructionPtr rec,
        FrameList::iterator A,
        FrameList::iterator B );

    osg::ref_ptr<osg::Node> createMapPointsNode( FrameList::iterator A, FrameList::iterator B );

    osg::ref_ptr<osg::Node> createDensePointsNode( FrameList::iterator A, FrameList::iterator B );

    osg::ref_ptr<osg::Node> createTrajectoryNode( FrameList::iterator A, FrameList::iterator B );

    osg::ref_ptr<osg::Node> createRigNode( FrameList::iterator A, FrameList::iterator B );

    static osg::ref_ptr<osg::Node> createRigNode(
        const Sophus::SE3d& leftcamera2rig,
        const Sophus::SE3d& rightcamera2rig);

    static osg::ref_ptr<osg::Node> createCameraNode();

    static void setPositionAttitude(osg::ref_ptr<osg::PositionAttitudeTransform> PAT, const Sophus::SE3d& se3);

protected:

    VisualizationDataPort* mVisualizationData;
    VisualizationSettingsPort* mVisualizationSettings;

    osg::ref_ptr<osg::Node> mRigNode;
    osg::ref_ptr<osg::Switch> mSegmentSwitch;
    std::vector<SegmentData> mSegments;
};

