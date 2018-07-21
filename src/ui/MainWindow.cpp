#include <QToolBar>
#include "ViewerWidget.h"
#include "MainWindow.h"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent)
{
    QToolBar* tb = addToolBar("Toolbar");
    tb->addAction("Quit");
    tb->addAction("About");

    m_viewer = new ViewerWidget();

    setCentralWidget(m_viewer);
    setWindowTitle("SLAM Demo");
}
