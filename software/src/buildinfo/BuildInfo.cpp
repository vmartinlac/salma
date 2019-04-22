#include "BuildInfo.h"

int BuildInfo::getVersionMajor()
{
    return SALMA_VERSION_MAJOR;
}

int BuildInfo::getVersionMinor()
{
    return SALMA_VERSION_MINOR;
}

int BuildInfo::getVersionRevision()
{
    return SALMA_VERSION_REVISION;
}

std::string BuildInfo::getBuildType()
{
    std::string tmp(SALMA_BUILD_TYPE);

    if(tmp.length() > 0)
    {
        return tmp;
    }
    else
    {
        return "N/A";
    }
}

std::string BuildInfo::getCompilationDate()
{
        return __DATE__ " " __TIME__;
}

std::string BuildInfo::getCompilerName()
{
        return SALMA_CXX_COMPILER_ID " " SALMA_CXX_COMPILER_VERSION;
}

std::string BuildInfo::getVersionString()
{
    return std::to_string(SALMA_VERSION_MAJOR) + "." + std::to_string(SALMA_VERSION_MINOR) + "." + std::to_string(SALMA_VERSION_REVISION);
}

std::string BuildInfo::getAsciiLogo()
{
    const std::string ret =
        "   _____         _      __  __          \n"
        "  / ____|  /\\   | |    |  \\/  |   /\\    \n"
        " | (___   /  \\  | |    | \\  / |  /  \\   \n"
        "  \\___ \\ / /\\ \\ | |    | |\\/| | / /\\ \\  \n"
        "  ____) / ____ \\| |____| |  | |/ ____ \\ \n"
        " |_____/_/    \\_\\______|_|  |_/_/    \\_\\\n"
        "                                        \n";

    return ret;
}

