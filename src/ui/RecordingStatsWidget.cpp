#include <QFormLayout>
#include "RecordingStatsWidget.h"
#include "RecordingParameters.h"

RecordingStatsWidget::RecordingStatsWidget(QWidget* parent) : QWidget(parent)
{
    mPort = new RecordingStatsInputPort(this);

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

    QObject::connect(mPort, SIGNAL(updated()), this, SLOT(refresh()));
}

RecordingStatsInputPort* RecordingStatsWidget::getPort()
{
    return mPort;
}

void RecordingStatsWidget::refresh()
{
    RecordingStatsInputData data;
    mPort->read(data);

    mLabelCamera->setText(data.camera_name.c_str());
    mLabelNumFrames->setText(QString::number(data.frame_count));
    mLabelOutputDirectory->setText(data.output_directory.c_str());
    mLabelResolution->setText( QString::number(data.image_width) + " x " + QString::number(data.image_height) );
}

