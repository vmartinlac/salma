#include "MainWindow.h"

MainWindow::MainWindow(QWidget* parent)
{
    ViewerWidget* w = new ViewerWidget();

    setCentralWidget(w);
    setWindowTitle("Salma Visualization");
}

MainWindow::~MainWindow()
{
}
