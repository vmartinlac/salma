#include <QApplication>
#include <iostream>
#include "Camera.h"
#include "Image.h"
#include "MainWindow.h"

int main(int num_args, char** args)
{
    QApplication app(num_args, args);

    CameraManager* camera_manager = CameraManager::createDefaultCameraManager();

    camera_manager->initialize();
    std::cout << camera_manager->getNumCameras() << std::endl;
    //Camera* cam = camera_manager->getDefaultCamera();
    camera_manager->finalize();

    delete camera_manager;

    return 0;
}
