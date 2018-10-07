#include <QFormLayout>
#include "CameraCalibrationStatsWidget.h"

CameraCalibrationStatsWidget::CameraCalibrationStatsWidget(QWidget* parent) : QWidget(parent)
{
    mPort = new CameraCalibrationStatsInputPort(this);

    QFormLayout* f = new QFormLayout;
    setLayout(f);

    mLabelCameraName = new QLabel("N/A");
    mLabelOutputPath = new QLabel("N/A");
    mLabelFrameCount = new QLabel("N/A");
    mLabelTrackingCount = new QLabel("N/A");

    f->addRow("Camera", mLabelCameraName);
    f->addRow("Output path", mLabelOutputPath);
    f->addRow("Frame count", mLabelFrameCount);
    f->addRow("Successful tracking attempts", mLabelTrackingCount);

    QObject::connect(mPort, SIGNAL(updated()), this, SLOT(refresh()));
}

CameraCalibrationStatsInputPort* CameraCalibrationStatsWidget::getPort()
{
    return mPort;
}

void CameraCalibrationStatsWidget::refresh()
{
    CameraCalibrationStatsInputData data;
    mPort->read(data);

    mLabelCameraName->setText(data.camera_name.c_str());
    mLabelOutputPath->setText(data.output_path.c_str());
    mLabelFrameCount->setText( QString::number(data.frame_count) );
    mLabelTrackingCount->setText( QString::number(data.successful_tracking_count) + "/" + QString::number(data.attempted_tracking_count) );
}

