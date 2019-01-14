#include <osg/ShapeDrawable>
#include <osg/LineWidth>
#include <osg/Geode>
#include <osg/PrimitiveSet>
#include <osg/Point>
#include "ViewerWidget.h"

ViewerWidget::ViewerWidget( SLAMReconstructionPtr reconstr, QWidget* parent ) : ViewerWidgetBase(parent)
{
    mReconstruction = reconstr;
    mShowRig = true;
    mShowTrajectory = true;
    mShowMapPoints = true;
    mShowDensePoints = true;
    mCurrentSegment = -1;
}

void ViewerWidget::setLighting(bool val)
{
    if( mSegmentSwitch )
    {
        if(val)
        {
            mSegmentSwitch->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON);
        }
        else
        {
            mSegmentSwitch->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        }
    }
}

void ViewerWidget::showSegment(int i)
{
    if( 0 <= i && i < mReconstruction->segments.size())
    {
        mCurrentSegment = i;
    }
    else
    {
        mCurrentSegment = -1;
    }

    updateSwitches();
}

void ViewerWidget::showMapPoints(bool val)
{
    mShowMapPoints = val;
    updateSwitches();
}

void ViewerWidget::showDensePoints(bool val)
{
    mShowDensePoints = val;
    updateSwitches();
}

void ViewerWidget::showRigs(bool val)
{
    mShowRig = val;
    updateSwitches();
}

void ViewerWidget::showTrajectory(bool val)
{
    mShowTrajectory = val;
    updateSwitches();
}

void ViewerWidget::buildScene()
{
    bool go_on = true;

    // clear previous scene.

    mSegmentSwitch = nullptr;
    mSegments.clear();
    mRigNode = nullptr;

    // check that we have a reconstruction.

    if(go_on)
    {
        go_on = bool(mReconstruction);
    }

    // create rig node.

    if(go_on)
    {
        mRigNode = createRigNode(
            mReconstruction->rig->cameras[0].camera_to_rig,
            mReconstruction->rig->cameras[1].camera_to_rig);
        go_on = (mRigNode != nullptr);
    }

    // create segment switch.

    if(go_on)
    {
        mSegmentSwitch = new osg::Switch();
        //mSegmentSwitch->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    }

    // create each segment.

    if(go_on)
    {
        for(int i=0; i<mReconstruction->segments.size(); i++)
        {
            addSegment(mReconstruction->segments[i]);

            go_on = ( i+1 == mSegmentSwitch->getNumChildren() && i+1 == mSegments.size() );
        }
    }

    if(go_on)
    {
        mShowRig = true;
        mShowTrajectory = true;
        mShowMapPoints = true;
        mShowDensePoints = true;
        mCurrentSegment = ( mReconstruction->segments.empty() ) ? -1 : 0;
        updateSwitches();
        setSceneData(mSegmentSwitch);
    }
    else
    {
        setSceneData(nullptr);
        mSegmentSwitch = nullptr;
        mSegments.clear();
        mRigNode = nullptr;
    }
}

void ViewerWidget::updateSwitches()
{
    if(mSegmentSwitch != nullptr)
    {
        mSegmentSwitch->setAllChildrenOff();

        if( mSegmentSwitch->getNumChildren() == mSegments.size() )
        {
            if( 0 <= mCurrentSegment && mCurrentSegment < mSegmentSwitch->getNumChildren() )
            {
                mSegmentSwitch->setSingleChildOn(mCurrentSegment);

                mSegments[mCurrentSegment].items->setValue( 0, mShowRig );
                mSegments[mCurrentSegment].items->setValue( 1, mShowTrajectory );
                mSegments[mCurrentSegment].items->setValue( 2, mShowMapPoints );
                mSegments[mCurrentSegment].items->setValue( 3, mShowDensePoints );
            }
        }
    }
}

osg::ref_ptr<osg::Node> ViewerWidget::createDensePointsNode(SLAMSegment& seg)
{
    return new osg::Group();
}

osg::ref_ptr<osg::Node> ViewerWidget::createRigNode(SLAMSegment& seg)
{
    osg::ref_ptr<osg::Group> grp = new osg::Group();

    for(SLAMFramePtr frame : seg.frames)
    {
        osg::ref_ptr<osg::PositionAttitudeTransform> PAT = new osg::PositionAttitudeTransform();
        PAT->addChild(mRigNode);
        setPositionAttitude(PAT, frame->frame_to_world);

        grp->addChild(PAT);
    }

    return grp;
}

osg::ref_ptr<osg::Node> ViewerWidget::createTrajectoryNode(SLAMSegment& seg)
{
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();
    osg::ref_ptr<osg::Vec3Array> colors = new osg::Vec3Array();
    osg::ref_ptr<osg::DrawElementsUInt> primitive = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES);

    int count = 0;
    for(SLAMFramePtr f : seg.frames)
    {
        const Sophus::SE3d left_camera_to_world = f->frame_to_world * mReconstruction->rig->cameras[0].camera_to_rig;
        const Sophus::SE3d right_camera_to_world = f->frame_to_world * mReconstruction->rig->cameras[1].camera_to_rig;

        vertices->push_back( osg::Vec3(
            left_camera_to_world.translation().x(),
            left_camera_to_world.translation().y(),
            left_camera_to_world.translation().z() ));

        vertices->push_back( osg::Vec3(
            right_camera_to_world.translation().x(),
            right_camera_to_world.translation().y(),
            right_camera_to_world.translation().z() ));

        primitive->addElement(2*count);
        primitive->addElement(2*count+1);
        if(count > 0)
        {
            primitive->addElement(2*(count-1));
            primitive->addElement(2*count);
            primitive->addElement(2*(count-1)+1);
            primitive->addElement(2*count+1);
        }

        count++;
    }

    colors->push_back( osg::Vec3(1.0, 0.0, 0.0) );

    geom->setVertexArray(vertices);
    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);
    geom->addPrimitiveSet(primitive);
    geom->getOrCreateStateSet()->setAttributeAndModes(new osg::LineWidth(3.0f));

    geode->addDrawable(geom);

    return geode;
}

osg::ref_ptr<osg::Node> ViewerWidget::createMapPointsNode( SLAMSegment& seg )
{
    struct IndexedMapPoint
    {
        int index_in_vertex_array;
        SLAMMapPointPtr mappoint;
    };

    std::map<int,IndexedMapPoint> indexed_mappoints;

    int num_mappoints = 0;

    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();
    osg::ref_ptr<osg::Vec3Array> mappoint_colors = new osg::Vec3Array();
    osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();

    // mappoints.

    for(SLAMFramePtr f : seg.frames)
    {
        for(SLAMView& v : f->views)
        {
            for(SLAMTrack& t : v.tracks)
            {
                if(t.mappoint)
                {
                    if( indexed_mappoints.count(t.mappoint->id) == 0 )
                    {
                        IndexedMapPoint& imp = indexed_mappoints[t.mappoint->id];
                        imp.mappoint = t.mappoint;
                        imp.index_in_vertex_array = num_mappoints;
                        num_mappoints++;

                        vertices->push_back( osg::Vec3( t.mappoint->position.x(), t.mappoint->position.y(), t.mappoint->position.z() ) );
                    }
                }
            }
        }
    }

    if( vertices->size() != num_mappoints ) throw std::runtime_error("internal error");

    mappoint_colors->push_back( osg::Vec3( 0.0, 1.0, 0.0 ) );

    geometry->setVertexArray(vertices);
    geometry->setColorArray(mappoint_colors);
    geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
    geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, num_mappoints));
    geometry->getOrCreateStateSet()->setAttributeAndModes(new osg::Point(3.0f));

    geode->addDrawable(geometry);

    return geode;
}

void ViewerWidget::addSegment(SLAMSegment& seg)
{
    osg::ref_ptr<osg::Node> node_mappoints = createMapPointsNode(seg);
    osg::ref_ptr<osg::Node> node_densepoints = createDensePointsNode(seg);
    osg::ref_ptr<osg::Node> node_trajectory = createTrajectoryNode(seg);
    osg::ref_ptr<osg::Node> node_rig = createRigNode(seg);

    osg::ref_ptr<osg::Switch> sw = new osg::Switch();
    sw->addChild(node_rig);
    sw->addChild(node_trajectory);
    sw->addChild(node_mappoints);
    sw->addChild(node_densepoints);
    sw->setAllChildrenOn();

    SegmentData s;
    s.items = sw;

    mSegmentSwitch->addChild(sw);
    mSegments.push_back(s);
}

osg::ref_ptr<osg::Node> ViewerWidget::createCameraNode()
{
    osg::ref_ptr<osg::Cone> cone = new osg::Cone();
    cone->setRotation(osg::Quat(M_PI, osg::Vec3(1.0, 0.0, 0.0)));

    osg::ref_ptr<osg::ShapeDrawable> drawable = new osg::ShapeDrawable(cone);
    drawable->setColor(osg::Vec4(0.75, 0.75, 0.75, 1.0));

    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    geode->addDrawable(drawable);

    return geode;
}

void ViewerWidget::setPositionAttitude(osg::ref_ptr<osg::PositionAttitudeTransform> PAT, const Sophus::SE3d& pose)
{
    PAT->setPosition( osg::Vec3(
        pose.translation().x(),
        pose.translation().y(),
        pose.translation().z() ));

    PAT->setAttitude( osg::Quat(
        pose.unit_quaternion().x(),
        pose.unit_quaternion().y(),
        pose.unit_quaternion().z(),
        pose.unit_quaternion().w() ));
}

osg::ref_ptr<osg::Node> ViewerWidget::createRigNode(const Sophus::SE3d& leftcamera2rig, const Sophus::SE3d& rightcamera2rig)
{
    const double baseline = ( rightcamera2rig.translation() - leftcamera2rig.translation() ).norm();
    const double scale = std::max(1.0e-4, 0.2 * baseline);

    osg::ref_ptr<osg::Group> grp = new osg::Group();
    osg::ref_ptr<osg::Node> camera = createCameraNode();
    osg::ref_ptr<osg::PositionAttitudeTransform> PAT_left = new osg::PositionAttitudeTransform();
    osg::ref_ptr<osg::PositionAttitudeTransform> PAT_right = new osg::PositionAttitudeTransform();

    PAT_left->addChild(camera);
    PAT_left->setScale( osg::Vec3(scale, scale, scale) );
    setPositionAttitude( PAT_left, leftcamera2rig );

    PAT_right->addChild(camera);
    PAT_right->setScale( osg::Vec3(scale, scale, scale) );
    setPositionAttitude( PAT_right, rightcamera2rig );

    grp->addChild(PAT_left);
    grp->addChild(PAT_right);

    return grp;
}

