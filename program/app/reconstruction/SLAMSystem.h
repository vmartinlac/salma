#pragma once

#include <memory>

class SLAMSystem
{
public:

    static SLAMSystem* instance();

    ~SLAMSystem();

    bool initialize(int num_args, char** args);

    void finalize();

protected:

    SLAMSystem();

    void printWelcomeMessage();

    bool parseCommandLineArguments(int num_args, char** args);
    
protected:

    static std::unique_ptr<SLAMSystem> mInstance;
};

