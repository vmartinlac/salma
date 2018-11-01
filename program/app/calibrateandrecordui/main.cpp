#include <QApplication>
#include <QMessageBox>
#include "VideoSystem.h"
#include "MainWindow.h"

/*
#include <QThread>
#include <thread>
#include <iostream>

void tmp()
{
    auto proc = [] ()
    {
        VideoSourcePtr cam = VideoSystem::instance()->createStereoAvtVideoSource(0, 1);

        for(int j=0; j<1; j++)
        {
            if(j > 0)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }

            cam->open();
            for(int i=0; i<1000; i++)
            {
                Image im;
                cam->trigger();
                //std::cout << "read!" << std::endl;
                cam->read(im);
                if(im.isValid() == false) std::cout << "invalid" << std::endl;
            }
            cam->close();
        }
    };

    VideoSystem::instance()->initialize(true);
    proc();
    std::cout << "switching to thread!" << std::endl;
    std::thread t(proc);
    t.join();
    VideoSystem::instance()->finalize();
    exit(0);
}
*/

int main(int num_args, char** args)
{
    //tmp();
    //
    QApplication app(num_args, args);

    if( VideoSystem::instance()->initialize(true) == false )
    {
        QMessageBox::critical(nullptr, "Error", "Could not initialize Vimba!");
    }
    else if( VideoSystem::instance()->getNumberOfAvtCameras() == 0 )
    {
        QMessageBox::critical(nullptr, "Error", "No camera were detected!");

        VideoSystem::instance()->finalize();
    }
    else
    {
        MainWindow* win = new MainWindow();

        win->show();

        app.exec();

        delete win;

        VideoSystem::instance()->finalize();
    }

    return 0;
}

