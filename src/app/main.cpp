#include <QApplication>
#include <iostream>
#include "Camera.h"
#include "SLAMEngine.h"
#include "MainWindow.h"

int main(int num_args, char** args)
{
    // initialize Qt.

    QApplication app(num_args, args);

    app.setApplicationName("slam");
    app.setOrganizationName("vmartinlac");

    // initialize camera manager and retrieve default camera.

    CameraManager* camera_manager = CameraManager::createVimbaCameraManager();
    camera_manager->initialize();

    Camera* camera = camera_manager->getDefaultCamera();
    std::cout << "Camera is " << camera->getHumanName() << std::endl;

    if( camera == nullptr )
    {
        std::cerr << "No camera available !" << std::endl;
        camera_manager->finalize();
        delete camera_manager;
        exit(0);
    }

    // create slam engine.

    SLAMEngine* slam = SLAMEngine::create(camera);

    // create main window.

    MainWindow* win = new MainWindow(slam);
    win->show();

    // exec the app.

    const int ret = app.exec();

    slam->requestInterruption();
    slam->wait();

    // clean up.

    delete win;
    delete slam;

    camera_manager->finalize();
    delete camera_manager;

    return ret;
}

