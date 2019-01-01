#include <QApplication>
#include <QCommandLineParser>
#include <QSqlDatabase>
#include "BuildInfo.h"
#include "VideoSystem.h"
#include "MainWindow.h"

int main(int num_args, char** args)
{
    Q_INIT_RESOURCE(db);

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

