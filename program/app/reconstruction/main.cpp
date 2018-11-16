#include <QApplication>
//#include <QCoreApplication>
#include <iostream>
#include <VideoSystem.h>
#include "BuildInfo.h"
#include "SLAMSystem.h"

int main(int num_args, char** args)
{
    QApplication app(num_args, args);
    app.setApplicationVersion(BuildInfo::getVersionString().c_str());

    if( VideoSystem::instance()->initialize() == false )
    {
        std::cerr << "Could not initialize video module!" << std::endl;
        exit(1);
    }

    SLAMSystem::instance()->run(num_args, args);

    VideoSystem::instance()->finalize();

    return 0;
}

