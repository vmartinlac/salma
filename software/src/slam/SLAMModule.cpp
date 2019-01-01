#include "SLAMModule.h"

SLAMModule::SLAMModule(SLAMContextPtr con)
{
    mContext = std::move(con);
}

SLAMModule::~SLAMModule()
{
}

SLAMContextPtr SLAMModule::context()
{
    return mContext;
}

bool SLAMModule::init()
{
    return true;
}

