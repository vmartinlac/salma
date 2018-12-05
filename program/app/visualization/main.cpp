#include <QApplication>
#include "MainWindow.h"

int main(int num_args, char** args)
{
    QApplication app(num_args, args);

    MainWindow* w = new MainWindow();
    w->show();

    const int ret = app.exec();

    return ret;
}

