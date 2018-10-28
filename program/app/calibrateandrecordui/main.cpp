#include <QApplication>
#include <QMessageBox>
#include "VideoSystem.h"
#include "MainWindow.h"

int main(int num_args, char** args)
{
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
