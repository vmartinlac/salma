#pragma once

#include <QMainWindow>

class ViewerWidget;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    MainWindow(QWidget* parent=nullptr);

protected:

    ViewerWidget* m_viewer;
};
