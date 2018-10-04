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

std::string BuildInfo::getReleaseName()
{
    return
        "BI " +
        std::to_string(BI_VERSION_MAJOR) +
        "." +
        std::to_string(BI_VERSION_MINOR) +
        "." +
        std::to_string(BI_VERSION_REVISION) +
        " " +
        std::string(BI_BUILD_TYPE) +
        " compiled on " __DATE__ " " __TIME__ " "
        "with " BI_CXX_COMPILER_ID " " BI_CXX_COMPILER_VERSION;
}

