#include <QToolBar>
#include <QSplitter>
#include <QLabel>
#include <QMessageBox>
#include "ViewerWidget.h"
#include "MainWindow.h"
#include "SLAMParametersDialog.h"

MainWindow::MainWindow(SLAMEngine* slam, QWidget* parent) :
    QMainWindow(parent),
    m_slam(slam)
{
    QToolBar* tb = addToolBar("Toolbar");
    m_a_start = tb->addAction("Start");
    m_a_stop = tb->addAction("Stop");

    m_a_parameters = tb->addAction("Parameters");

    QAction* a_about = tb->addAction("About");

    connect(m_a_parameters, SIGNAL(triggered()), this, SLOT(ask_slam_parameters()));

    connect(m_a_start, SIGNAL(triggered()), this, SLOT(start_slam()));
    connect(m_a_stop, SIGNAL(triggered()), this, SLOT(stop_slam()));
    connect(a_about, SIGNAL(triggered()), this, SLOT(about()));

    connect(m_slam, SIGNAL(started()), this, SLOT(slam_started()));
    connect(m_slam, SIGNAL(finished()), this, SLOT(slam_stopped()));

    slam_stopped();

    m_viewer = new ViewerWidget();
    m_stats = new StatsWidget( m_slam->getOutput() );
    m_video = new VideoWidget( m_slam->getOutput() );

    QSplitter* outer_splitter = new QSplitter();
    QSplitter* inner_splitter = new QSplitter();

    outer_splitter->setOrientation(Qt::Horizontal);
    outer_splitter->setChildrenCollapsible(false);
    inner_splitter->setOrientation(Qt::Vertical);
    outer_splitter->setChildrenCollapsible(false);

    outer_splitter->addWidget(m_viewer);
    outer_splitter->addWidget(inner_splitter);
    inner_splitter->addWidget(m_video);
    inner_splitter->addWidget(m_stats);

    setCentralWidget(outer_splitter);
    setWindowTitle("SLAM Demo");
    resize(800, 600);
}

void MainWindow::ask_slam_parameters()
{
    SLAMParametersDialog::ask(this, m_slam_parameters);
}

void MainWindow::about()
{
    QMessageBox::information(this, "About", "Victor Martin Lac 2018");
}

void MainWindow::start_slam()
{
    if( m_slam->isRunning() == false )
    {
        m_a_parameters->setEnabled(false);
        m_slam->start();
    }
}

void MainWindow::stop_slam()
{
    if( m_slam->isRunning() )
    {
        m_slam->requestInterruption();
    }
}

void MainWindow::slam_started()
{
    m_a_start->setEnabled(false);
    //m_a_start->setVisible(false);
    m_a_stop->setEnabled(true);
    m_a_parameters->setEnabled(false);
    //m_a_stop->setVisible(true);
}

void MainWindow::slam_stopped()
{
    m_a_start->setEnabled(true);
    //m_a_start->setVisible(true);
    m_a_stop->setEnabled(false);
    m_a_parameters->setEnabled(true);
    //m_a_stop->setVisible(false);
}

