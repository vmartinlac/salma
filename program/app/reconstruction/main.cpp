#include <QCoreApplication>
#include <iostream>
#include <VideoSystem.h>
#include "BuildInfo.h"
#include "SLAMSystem.h"

int main(int num_args, char** args)
{
    QCoreApplication app(num_args, args);
    app.setApplicationVersion(BuildInfo::getVersionString().c_str());

    if( VideoSystem::instance()->initialize(false) == false )
    {
        std::cerr << "Could not initialize video module!" << std::endl;
        exit(1);
    }

    if( SLAMSystem::instance()->initialize(num_args, args) == false )
    {
        std::cerr << "Could not initialize SLAM system!" << std::endl;
        exit(1);
    }

    SLAMSystem::instance()->run();

    SLAMSystem::instance()->finalize();
    VideoSystem::instance()->finalize();

    return 0;
}

