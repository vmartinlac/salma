#include <QApplication>
#include <iostream>
#include "Camera.h"
#include "SLAMEngine.h"
#include "MainWindow.h"

int main(int num_args, char** args)
{
    QApplication app(num_args, args);

    // initialize camera manager and retrieve default camera.
    CameraManager* camera_manager = CameraManager::createVimbaCameraManager();
    camera_manager->initialize();
    Camera* camera = camera_manager->getDefaultCamera();
    if( camera == nullptr )
    {
        /*
        std::cerr << "No camera available !" << std::endl;
        camera_manager->finalize();
        delete camera_manager;
        exit(0);
        */
    }

    // create slam engine.
    SLAMEngine* slam = SLAMEngine::create(camera);

    MainWindow* win = new MainWindow(slam);
    win->show();

    const int ret = app.exec();

    delete win;
    delete slam;

    // release camera manager.
    camera_manager->finalize();
    delete camera_manager;

    return 0;
}

