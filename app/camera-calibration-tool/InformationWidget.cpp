#include <QFormLayout>
#include "InformationWidget.h"
#include "Parameters.h"

InformationWidget::InformationWidget(Output* output, QWidget* parent) : QWidget(parent)
{
    mOutput = output;

    QFormLayout* f = new QFormLayout;
    setLayout(f);

    mLabelCamera = new QLabel("N/A");
    mLabelOutputDirectory = new QLabel("N/A");
    mLabelNumFrames = new QLabel("N/A");
    mLabelResolution = new QLabel("N/A");

    f->addRow("Camera", mLabelCamera);
    f->addRow("Output directory", mLabelOutputDirectory);
    f->addRow("Num frames written", mLabelNumFrames);
    f->addRow("Resolution", mLabelResolution);

    QObject::connect(output, SIGNAL(updated()), this, SLOT(refresh()));
}

void InformationWidget::refresh()
{
    OutputData data;
    mOutput->read(data);

    mLabelCamera->setText(data.camera_name);
    mLabelNumFrames->setText(QString::number(data.frame_count));
    mLabelOutputDirectory->setText(data.output_directory);
    mLabelResolution->setText( QString::number(data.image.cols) + " x " + QString::number(data.image.rows) );
}

