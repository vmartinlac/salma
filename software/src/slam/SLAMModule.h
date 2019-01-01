#pragma once

#include "SLAMContext.h"

class SLAMModule
{
public:

    SLAMModule(SLAMContextPtr con);
    virtual ~SLAMModule();

    virtual bool init();

    virtual void operator()() = 0;

    SLAMContextPtr context();

private:

    SLAMContextPtr mContext;
};

typedef std::shared_ptr<SLAMModule> SLAMModulePtr;

