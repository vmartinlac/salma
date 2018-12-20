#include <QApplication>
#include "MainWindow.h"

int main(int num_args, char** args)
{
    QApplication app(num_args, args);

    MainWindow* w = new MainWindow();

    w->show();

    app.exec();

    return 0;
}

