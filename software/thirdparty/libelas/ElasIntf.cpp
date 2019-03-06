#include "ElasIntfImpl.h"
#include "ElasIntf.h"

cv::Ptr<ElasIntf> ElasIntf::create()
{
    return new ElasIntfImpl();
}

