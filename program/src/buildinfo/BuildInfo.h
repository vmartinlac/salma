#pragma once

#include <string>

class BuildInfo
{
public:
    static int getVersionMajor();
    static int getVersionMinor();
    static int getVersionRevision();
    static std::string getVersionString();

    static std::string getBuildType();
    static std::string getCompilerName();
    static std::string getCompilationDate();
};

