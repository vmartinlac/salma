#include <QCoreApplication>
#include <iostream>
#include "SLAMSystem.h"

int main(int num_args, char** args)
{
    QCoreApplication app(num_args, args);

    const bool ok = SLAMSystem::instance()->initialize(num_args, args);

    if(ok)
    {
        SLAMSystem::instance()->run();

        SLAMSystem::instance()->finalize();
    }
    else
    {
        std::cerr << "Could not initialize SLAM system!" << std::endl;
    }

    return 0;
}

