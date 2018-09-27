#include <QApplication>
#include <iostream>
#include "Camera.h"
#include "SLAMEngine.h"
#include "MainWindow.h"
#include "VimbaCamera.h"

int main(int num_args, char** args)
{
    QApplication app(num_args, args);
    SLAMEngine* slam = nullptr;
    MainWindow* win = nullptr;
    int ret = 0;

    app.setApplicationName("slam");
    app.setOrganizationName("vmartinlac");

    // initialize Vimba.

    if( VimbaCameraManager::instance().initialize() == false )
    {
        std::cerr << "Could not initialize Vimba!" << std::endl;
    }

    slam = SLAMEngine::create();
    win = new MainWindow(slam);
    win->show();
    ret = app.exec();

    delete win;

    slam->requestInterruption();
    slam->wait();
    delete slam;

    VimbaCameraManager::instance().finalize();

    return ret;
}

