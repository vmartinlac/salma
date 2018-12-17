#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osg/PrimitiveSet>
#include <osg/Point>
#include "ViewerWidget.h"

ViewerWidget::ViewerWidget(
    VisualizationDataPort* visudata,
    VisualizationSettingsPort* visusettings,
    QWidget* parent) : ViewerWidgetBase(parent)
{
    mVisualizationSettings = visusettings;
    mVisualizationData = visudata;

    connect(mVisualizationData, SIGNAL(updated()), this, SLOT(buildScene()));
    connect(mVisualizationSettings, SIGNAL(updated()), this, SLOT(applyVisualizationSettings()));

    buildScene();
}

void ViewerWidget::buildScene()
{
    mVisualizationData->beginRead();

    VisualizationData& vd = mVisualizationData->data();

    bool go_on = true;

    // clear previous scene.

    mSegmentSwitch = nullptr;
    mSegments.clear();
    mRigNode = nullptr;

    // check that we have a reconstruction.

    if(go_on)
    {
        go_on = bool(vd.reconstruction);
    }

    // create rig node.

    if(go_on)
    {
        mRigNode = createRigNode( vd.reconstruction->left_camera_to_rig, vd.reconstruction->right_camera_to_rig );
        go_on = (mRigNode != nullptr);
    }

    // create segment switch.

    if(go_on)
    {
        mSegmentSwitch = new osg::Switch();
        //mSegmentSwitch->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        mSegmentSwitch->getOrCreateStateSet()->setAttributeAndModes(new osg::Point(3.0f));
    }

    // create each segment.

    if(go_on)
    {
        ReconstructionPtr rec = vd.reconstruction;

        int count = 0;
        for( std::pair<FrameList::iterator,FrameList::iterator>& pair : vd.segments )
        {
            addSegment( rec, pair.first, pair.second );
            count++;

            if( mSegments.size() != count || mSegmentSwitch->getNumChildren() != count ) throw std::runtime_error("internal error");
        }
    }

    if(go_on)
    {
        mSegmentSwitch->setSingleChildOn(0);
        _viewer->setSceneData(mSegmentSwitch);
    }
    else
    {
        _viewer->setSceneData(nullptr);
        mSegmentSwitch = nullptr;
        mSegments.clear();
        mRigNode = nullptr;
    }

    mVisualizationData->endRead();
}

void ViewerWidget::applyVisualizationSettings()
{
}

osg::ref_ptr<osg::Node> ViewerWidget::createDensePointsNode( FrameList::iterator A, FrameList::iterator B )
{
    return new osg::Group();
}

osg::ref_ptr<osg::Node> ViewerWidget::createRigNode( FrameList::iterator A, FrameList::iterator B )
{
    osg::ref_ptr<osg::Group> grp = new osg::Group();

    for( FrameList::iterator it = A; it!=B; it++)
    {
        FramePtr frame = *it;

        osg::ref_ptr<osg::PositionAttitudeTransform> PAT = new osg::PositionAttitudeTransform();
        PAT->addChild(mRigNode);
        setPositionAttitude(PAT, frame->frame_to_world);

        grp->addChild(PAT);
    }

    return grp;
}

osg::ref_ptr<osg::Node> ViewerWidget::createTrajectoryNode( FrameList::iterator A, FrameList::iterator B )
{
    return new osg::Group();
    /*
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();
    osg::ref_ptr<osg::Vec3Array> colors = new osg::Vec3Array();
    osg::ref_ptr<osg::DrawElementsUInt> primitive = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES);

    int count = 0;
    for(FrameList::iterator it=A; it!=B; it++)
    {
        ;

        count++;
    }

    geom->setVertexArray(vertices);
    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);
    geom->addPrimitiveSet(primitive);

    geode->addDrawable(geom);

    return geode;
    */
}

osg::ref_ptr<osg::Node> ViewerWidget::createMapPointsNode( FrameList::iterator A, FrameList::iterator B )
{
/*
    struct IndexedMapPoint
    {
        int index_in_vertex_array;
        MapPointPtr mappoint;
    };

    std::map<int,IndexedMapPoint> indexed_mappoints;

    int num_mappoints = 0;

    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();

    // mappoints.

    for(FrameList::iterator it=A; it!=B; it++)
    {
        FramePtr f = *it;

        for(View& v : f->views)
        {
            for(Projection& p : v.projections)
            {
                if(p.mappoint)
                {
                    if( indexed_mappoints.count(p.mappoint->id) == 0 )
                    {
                        IndexedMapPoint& imp = indexed_mappoints[p.mappoint->id];
                        imp.mappoint = p.mappoint;
                        imp.index_in_vertex_array = num_mappoints;
                        num_mappoints++;

                        vertices->push_back( osg::Vec3( p.mappoint->position.x(), p.mappoint->position.y(), p.mappoint->position.z() ) );
                    }
                }
            }
        }
    }

    osg::ref_ptr<osg::Vec3Array> mappoint_colors = new osg::Vec3Array();
    mappoint_colors->push_back( osg::Vec3( 0.0, 1.0, 0.0 ) );

    osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();
    geometry->setVertexArray(vertices);
    geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, num_mappoints));
    geometry->setColorArray(mappoint_colors);
    geometry->setColorBinding(osg::Geometry::BIND_OVERALL);

    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    geode->addDrawable(geometry);

    osg::ref_ptr<osg::Switch> s = new osg::Switch();
    s->addChild(geode, true);

    mSegmentSwitch->addChild(s, true);

    //mSegments.push_back(s);
*/
    //return mRigNode;
    return new osg::Group();
}

void ViewerWidget::addSegment( ReconstructionPtr rec, FrameList::iterator A, FrameList::iterator B )
{
    osg::ref_ptr<osg::Node> node_mappoints = createMapPointsNode(A, B);
    osg::ref_ptr<osg::Node> node_densepoints = createDensePointsNode(A, B);
    osg::ref_ptr<osg::Node> node_trajectory = createTrajectoryNode(A, B);
    osg::ref_ptr<osg::Node> node_rig = createRigNode(A, B);

    osg::ref_ptr<osg::Switch> sw = new osg::Switch();
    sw->addChild(node_mappoints);
    sw->addChild(node_trajectory);
    sw->addChild(node_densepoints);
    sw->addChild(node_rig);
    sw->setAllChildrenOn();

    SegmentData s;
    s.map[ITEM_MAPPOINTS] = 0;
    s.map[ITEM_TRAJECTORY] = 1;
    s.map[ITEM_DENSEPOINTS] = 2;
    s.map[ITEM_RIG] = 3;
    s.items = sw;

    mSegments.push_back(s);

    mSegmentSwitch->addChild(sw);
}

osg::ref_ptr<osg::Node> ViewerWidget::createCameraNode()
{
    osg::ref_ptr<osg::Cone> cone = new osg::Cone();

    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    geode->addDrawable(new osg::ShapeDrawable(cone));

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

