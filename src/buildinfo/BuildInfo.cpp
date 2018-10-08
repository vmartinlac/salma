#include "BuildInfo.h"

int BuildInfo::getVersionMajor()
{
    return BI_VERSION_MAJOR;
}

int BuildInfo::getVersionMinor()
{
    return BI_VERSION_MINOR;
}

int BuildInfo::getVersionRevision()
{
    return BI_VERSION_REVISION;
}

std::string BuildInfo::getBuildType()
{
    return std::string(BI_BUILD_TYPE);
}

std::string BuildInfo::getCompilationDate()
{
        return __DATE__ " " __TIME__;
}

std::string BuildInfo::getCompilerName()
{
        return BI_CXX_COMPILER_ID " " BI_CXX_COMPILER_VERSION;
}

