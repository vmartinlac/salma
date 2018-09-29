#pragma once

#include <QWidget>
#include <QLabel>

class InformationWidget : public QWidget
{
public:

    InformationWidget(QWidget* parent=nullptr);

protected:

    QLabel* mLabelCamera;
    QLabel* mLabelOutputDirectory;
    QLabel* mLabelNumFrames;
};

