#include <iostream>
#include <QApplication>
#include <QCommandLineParser>
#include <QSqlDatabase>
#include "BuildInfo.h"
#include "VideoSystem.h"
#include "MainWindow.h"

void printWelcomeMessage()
{
    const std::string logo = BuildInfo::getAsciiLogo();

    std::cout << logo << std::endl;
    std::cout << "Version: " << BuildInfo::getVersionMajor() << "." << BuildInfo::getVersionMinor() << "." << BuildInfo::getVersionRevision() << std::endl;
    std::cout << "Author: Victor MARTIN-LAC 2018-2019" << std::endl;
    std::cout << "Build date: " << BuildInfo::getCompilationDate() << std::endl;
    std::cout << "Compiler: " << BuildInfo::getCompilerName() << std::endl;
    std::cout << std::endl;
}

int main(int num_args, char** args)
{
    Q_INIT_RESOURCE(db);

    printWelcomeMessage();

    QApplication app(num_args, args);
    app.setApplicationName("SALMA");
    app.setOrganizationName("Victor Martin Lac");
    app.setApplicationVersion(BuildInfo::getVersionString().c_str());

    QCommandLineParser parser;
    parser.addPositionalArgument("[PROJECT]", "Path to project");
    parser.addHelpOption();
    parser.addVersionOption();
    parser.process(app);

    if(parser.positionalArguments().size() > 1)
    {
        std::cout << "Bad command line!" << std::endl;
        exit(1);
    }

    QSqlDatabase::addDatabase("QSQLITE");

    VideoSystem::instance()->initialize();

    MainWindow* w = new MainWindow();
    w->show();

    if(parser.positionalArguments().size() == 1)
    {
        const QString path = parser.positionalArguments().front();

        w->loadProjectGivenOnCommandLine(path);

        /*
        QMetaObject::invokeMethod(
            w,
            "loadProjectGivenOnCommandLine",
            Qt::QueuedConnection,
            Q_ARG(const QString&, path));
        */
    }

    const int ret = app.exec();

    delete w;

    VideoSystem::instance()->finalize();

    return ret;
}

