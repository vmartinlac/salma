#include "FinitePriorityQueue.h"
#include "SLAMModule1KFS.h"

SLAMModule1KFS::SLAMModule1KFS(SLAMContextPtr con) :
    SLAMModule(SLAM_MODULE1_KEYFRAMESELECTION, con)
{
}

SLAMModule1KFS::~SLAMModule1KFS()
{
}

bool SLAMModule1KFS::init()
{
    SLAMContextPtr con = context();

    mTranslationThreshold = con->configuration->kfs.translation_threshold;
    mAngleThreshold = con->configuration->kfs.angle_threshold;

    return true;
}

SLAMModuleResult SLAMModule1KFS::operator()()
{
    std::cout << "   KEYFRAME SELECTION" << std::endl;

    SLAMReconstructionPtr rec = context()->reconstruction;

    const int N = rec->frames.size();

    bool is_keyframe = true;

    if( rec->frames[N-1]->aligned_wrt_previous_frame )
    {
        const Sophus::SE3d prev_to_world = rec->frames[N-2]->frame_to_world;
        const Sophus::SE3d curr_to_world = rec->frames[N-1]->frame_to_world;

        const double alpha = ( curr_to_world.translation() - prev_to_world.translation() ).norm();
        const double beta = curr_to_world.unit_quaternion().angularDistance( prev_to_world.unit_quaternion() );

        is_keyframe = (alpha > mTranslationThreshold || beta > mAngleThreshold);
    }
    else
    {
        is_keyframe = true;
    }

    std::cout << "      Is keyframe: " << ( (is_keyframe) ? "yes" : "no" ) << std::endl;

    if(is_keyframe)
    {
        return SLAMModuleResult(false, SLAM_MODULE1_LOCALBUNDLEADJUSTMENT);
    }
    else
    {
        rec->frames.pop_back();
        return SLAMModuleResult(true, SLAM_MODULE1_FEATURES);
    }
}

