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
    mSegmentSwitch = new osg::Switch();
    mSegmentSwitch->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    mSegmentSwitch->getOrCreateStateSet()->setAttributeAndModes(new osg::Point(3.0f));

    mVisualizationData->beginRead();

    for( std::pair<FrameList::iterator,FrameList::iterator>& pair : mVisualizationData->data().segments )
    {
        buildSegment( pair.first, pair.second );
    }

    mVisualizationData->endRead();

    mSegmentSwitch->setSingleChildOn(0);

    _viewer->setSceneData(mSegmentSwitch);
}

void ViewerWidget::applyVisualizationSettings()
{
}

void ViewerWidget::buildSegment( FrameList::iterator A, FrameList::iterator B )
{
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
}


//////////:

/*
osg::ref_ptr<osg::Node> ViewerWidget::createCameraNode()
{
    static double camera_points[3*8] =
    {
        -1.0, -1.0, 0.0,
        1.0, -1.0, 0.0,
        1.0, 1.0, 0.0,
        -1.0, 1.0, 0.0,
        -2.0, -2.0, 0.5,
        2.0, -2.0, 0.5,
        2.0, 2.0, 0.5,
        -2.0, 2.0, 0.5,
    };

    static unsigned int triangles[] =
    {
        0, 1, 2,
        2, 3, 0
    };

    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();
    osg::ref_ptr<osg::Vec3Array> colors = new osg::Vec3Array();
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    osg::ref_ptr<osg::DrawElementsUInt> primitive = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES);

    for(int i=0; i<8; i++)
    {
        vertices->push_back( osg::Vec3(
            camera_points[3*i+0],
            camera_points[3*i+1],
            camera_points[3*i+2] ));
    }

    for(int i=0; i<3*2; i++)
    {
        primitive->addElement(triangles[i]);
    }

    colors->push_back( osg::Vec3(1.0, 0.0, 0.0) );

    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);
    geom->setVertexArray(vertices);
    geom->addPrimitiveSet(primitive);

    geode->addDrawable(geom);

    return geode;
}

osg::ref_ptr<osg::Node> ViewerWidget::createRigNode(const Sophus::SE3d& leftcamera2world, const Sophus::SE3d& rightcamera2world)
{
    osg::ref_ptr<osg::Group> grp = new osg::Group();

    const double baseline = ( rightcamera2world.translation() - leftcamera2world.translation() ).norm();
    const double scale = 1.0 * baseline;

    osg::ref_ptr<osg::Node> camera = createCameraNode();
    osg::ref_ptr<osg::PositionAttitudeTransform> PAT_left = new osg::PositionAttitudeTransform();
    osg::ref_ptr<osg::PositionAttitudeTransform> PAT_right = new osg::PositionAttitudeTransform();

    PAT_left->addChild(camera);
    PAT_left->setScale( osg::Vec3(scale, scale, scale) );
    PAT_left->setPosition( osg::Vec3(
        leftcamera2world.translation().x(),
        leftcamera2world.translation().y(),
        leftcamera2world.translation().z() ));
    PAT_left->setAttitude( osg::Quat(
        leftcamera2world.unit_quaternion().x(),
        leftcamera2world.unit_quaternion().y(),
        leftcamera2world.unit_quaternion().z(),
        leftcamera2world.unit_quaternion().w() ));

    PAT_right->addChild(camera);
    PAT_right->setScale( osg::Vec3(scale, scale, scale) );
    PAT_right->setPosition( osg::Vec3(
        rightcamera2world.translation().x(),
        rightcamera2world.translation().y(),
        rightcamera2world.translation().z() ));
    PAT_right->setAttitude( osg::Quat(
        rightcamera2world.unit_quaternion().x(),
        rightcamera2world.unit_quaternion().y(),
        rightcamera2world.unit_quaternion().z(),
        rightcamera2world.unit_quaternion().w() ));

    return grp;
}

*/

