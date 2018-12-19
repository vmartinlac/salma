#include <QMenu>
#include <QStatusBar>
#include <QListWidget>
#include <QToolBar>
#include <QSplitter>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTextEdit>
#include <QTabWidget>
#include <QMenuBar>
#include <QAction>
#include <QApplication>
#include <QMainWindow>

class PanelReconstruction : public QWidget
{

public:

    PanelReconstruction(QWidget* parent=nullptr)
    {
        QToolBar* tb = new QToolBar();
        tb->addAction("New");
        tb->addAction("Inspect");
        tb->addAction("Rename");
        tb->addAction("Delete");

        QSplitter* splitter = new QSplitter();
        splitter->addWidget(new QListWidget());
        splitter->addWidget(new QTextEdit());

        QVBoxLayout* lay = new QVBoxLayout();
        lay->addWidget(tb);
        lay->addWidget(splitter);

        setLayout(lay);
    }
};

class PanelRecording : public QWidget
{

public:

    PanelRecording(QWidget* parent=nullptr)
    {
        QToolBar* tb = new QToolBar();
        tb->addAction("New");
        tb->addAction("Play");
        tb->addAction("Rename");
        tb->addAction("Delete");

        QSplitter* splitter = new QSplitter();
        splitter->addWidget(new QListWidget());
        splitter->addWidget(new QTextEdit());

        QVBoxLayout* lay = new QVBoxLayout();
        lay->addWidget(tb);
        lay->addWidget(splitter);

        setLayout(lay);
    }
};
class PanelRig : public QWidget
{

public:

    PanelRig(QWidget* parent=nullptr)
    {
        QToolBar* tb = new QToolBar();
        tb->addAction("New");
        tb->addAction("Import");
        tb->addAction("Export");
        tb->addAction("Rename");
        tb->addAction("Delete");

        QSplitter* splitter = new QSplitter();
        splitter->addWidget(new QListWidget());
        splitter->addWidget(new QTextEdit());

        QVBoxLayout* lay = new QVBoxLayout();
        lay->addWidget(tb);
        lay->addWidget(splitter);

        setLayout(lay);
    }
};

class PanelCamera : public QWidget
{

public:

    PanelCamera(QWidget* parent=nullptr)
    {
        QToolBar* tb = new QToolBar();
        tb->addAction("New");
        tb->addAction("Import");
        tb->addAction("Export");
        tb->addAction("Rename");
        tb->addAction("Delete");

        QSplitter* splitter = new QSplitter();
        splitter->addWidget(new QListWidget());
        splitter->addWidget(new QTextEdit());

        QVBoxLayout* lay = new QVBoxLayout();
        lay->addWidget(tb);
        lay->addWidget(splitter);

        setLayout(lay);
    }
};

class MainWindow : public QMainWindow
{
public:

    MainWindow(QWidget* parent=nullptr) : QMainWindow(parent)
    {
        QMenu* menuProject = menuBar()->addMenu("Project");
        menuProject->addAction("Information");
        menuProject->addAction("Clear");
        menuProject->addAction("Quit");

        QMenu* menuHelp = menuBar()->addMenu("Help");
        menuHelp->addAction("About");

        QTabWidget* tab = new QTabWidget();
        tab->addTab(new PanelCamera(), "Camera");
        tab->addTab(new PanelRig(), "Camera rig");
        tab->addTab(new PanelRecording(), "Recording");
        tab->addTab(new PanelReconstruction(), "Reconstruction");

        statusBar()->showMessage("SALMA v1.0");

        setCentralWidget(tab);
        setWindowTitle("Salma");
    }

protected:

    ;
};

int main(int num_args, char** args)
{
    QApplication app(num_args, args);

    MainWindow* w = new MainWindow();

    w->show();

    app.exec();

    return 0;
}

