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
        osg::ref_ptr<osg::Switch> visualization_items_switch;
    };

protected:

    void buildSegment(
        FrameList::iterator A,
        FrameList::iterator B);

protected:

    VisualizationDataPort* mVisualizationData;
    VisualizationSettingsPort* mVisualizationSettings;
    osg::ref_ptr<osg::Switch> mSegmentSwitch;
    std::vector<SegmentWrapper> mSegments;
};

