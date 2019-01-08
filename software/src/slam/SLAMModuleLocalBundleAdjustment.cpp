#include "SLAMModuleLocalBundleAdjustment.h"

SLAMModuleLocalBundleAdjustment::SLAMModuleLocalBundleAdjustment(SLAMContextPtr con) : SLAMModule(con)
{
}

SLAMModuleLocalBundleAdjustment::~SLAMModuleLocalBundleAdjustment()
{
}

bool SLAMModuleLocalBundleAdjustment::init()
{
    SLAMContextPtr con = context();

    //const double scale_factor = con->configuration->features_scale_factor;

    return true;
}

void SLAMModuleLocalBundleAdjustment::operator()()
{
    std::cout << "   LOCAL BUNDLE ADJUSTMENT" << std::endl;

    SLAMReconstructionPtr reconstr = context()->reconstruction;

    if( reconstr->frames.empty() ) throw std::runtime_error("internal error");

    SLAMFramePtr frame = reconstr->frames.back();

    /*
    if( context()->configuration->localbundleadjustment_debug )
    {
    }

    std::cout << "      Num keypoints on left view: " << frame->views[0].keypoints.size() << std::endl;
    */
}

