#include <QApplication>
#include <iostream>
#include "Camera.h"
#include "Image.h"
#include "MainWindow.h"
#include "SLAMEngine.h"

int main(int num_args, char** args)
{
    QApplication app(num_args, args);

    MainWindow* w = new MainWindow();

    w->show();

    app.exec();

    delete w;

    return 0;

    ///////

    CameraManager* camera_manager = CameraManager::createDefaultCameraManager();

    camera_manager->initialize();

    Camera* cam = camera_manager->getDefaultCamera();

    if(cam != nullptr)
    {
        SLAMEngine* engine = SLAMEngine::createDefaultSLAMEngine();

        engine->initialize();

        cam->start();

        while(true)
        {
            Image* image = cam->readImage();
            engine->processNextView(image);
            delete image;
        }

        cam->stop();

        delete engine;
    }

    camera_manager->finalize();

    delete camera_manager;

    return 0;
}
