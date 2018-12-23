#include <QApplication>
#include <QSqlDatabase>
#include "BuildInfo.h"
#include "VideoSystem.h"
#include "MainWindow.h"

int main(int num_args, char** args)
{
    QApplication app(num_args, args);
    app.setOrganizationName("vmartinlac");
    app.setApplicationVersion(BuildInfo::getVersionString().c_str());

    QSqlDatabase::addDatabase("QSQLITE");

    VideoSystem::instance()->initialize();

    MainWindow* w = new MainWindow();
    w->show();

    const int ret = app.exec();

    delete w;

    VideoSystem::instance()->finalize();

    return ret;
}

