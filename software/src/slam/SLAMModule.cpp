#include "SLAMModule.h"

SLAMModule::SLAMModule(SLAMModuleId id, SLAMContextPtr con)
{
    mId = id;
    mContext = std::move(con);
}

SLAMModule::~SLAMModule()
{
}

bool SLAMModule::init()
{
    return true;
}

