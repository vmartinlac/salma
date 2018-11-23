#include <Eigen/Eigen>
#include "SLAMModuleAlignment.h"

SLAMModuleAlignment::SLAMModuleAlignment(SLAMProjectPtr project) : SLAMModule(project)
{
    //mMaxNumberOfPreviousFrames = project->getParameterInteger("max_number_of_previous_frames", 5);
}

void SLAMModuleAlignment::run(FramePtr frame)
{
    if( frame->previous_frame )
    {
    }
}

