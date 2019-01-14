#pragma once

#include <osg/Switch>
#include "ViewerWidgetBase.h"
#include "SLAMDataStructures.h"

class ViewerWidget : public ViewerWidgetBase
{
    Q_OBJECT

public:

    ViewerWidget(SLAMReconstructionPtr reconstr, QWidget* parent=nullptr);

    void buildScene();

public slots:

    void showMapPoints(bool);
    void showDensePoints(bool);
    void showRigs(bool);
    void showTrajectory(bool);
    void showSegment(int i);
    void setLighting(bool);

protected:

    struct SegmentData
    {
        osg::ref_ptr<osg::Switch> items;
    };

protected:

    void updateSwitches();

    void addSegment(SLAMSegment& s);

    osg::ref_ptr<osg::Node> createMapPointsNode(SLAMSegment& seg);
    osg::ref_ptr<osg::Node> createDensePointsNode(SLAMSegment& seg);
    osg::ref_ptr<osg::Node> createTrajectoryNode(SLAMSegment& seg);
    osg::ref_ptr<osg::Node> createRigNode(SLAMSegment& seg);

    static osg::ref_ptr<osg::Node> createRigNode(
        const Sophus::SE3d& leftcamera2rig,
        const Sophus::SE3d& rightcamera2rig);

    static osg::ref_ptr<osg::Node> createCameraNode();

    static void setPositionAttitude(osg::ref_ptr<osg::PositionAttitudeTransform> PAT, const Sophus::SE3d& se3);

protected:

    osg::ref_ptr<osg::Node> mRigNode;
    osg::ref_ptr<osg::Switch> mSegmentSwitch;
    std::vector<SegmentData> mSegments;

    int mCurrentSegment;
    bool mShowRig;
    bool mShowTrajectory;
    bool mShowMapPoints;
    bool mShowDensePoints;
    SLAMReconstructionPtr mReconstruction;
};

