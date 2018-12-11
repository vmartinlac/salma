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

