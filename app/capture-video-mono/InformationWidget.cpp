#include <QFormLayout>
#include "InformationWidget.h"

InformationWidget::InformationWidget(QWidget* parent) : QWidget(parent)
{
    QFormLayout* f = new QFormLayout;
    setLayout(f);

    mLabelCamera = new QLabel("N/A");
    mLabelOutputDirectory = new QLabel("N/A");
    mLabelNumFrames = new QLabel("N/A");

    f->addRow("Camera", mLabelCamera);
    f->addRow("Output directory", mLabelOutputDirectory);
    f->addRow("Num frames written", mLabelNumFrames);
}

