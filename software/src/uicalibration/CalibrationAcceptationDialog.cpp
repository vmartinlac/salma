#include <QJsonDocument>
#include <QJsonObject>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include "CalibrationAcceptationDialog.h"

CalibrationAcceptationDialog::CalibrationAcceptationDialog(QWidget* parent) : QDialog(parent)
{
    mText = new QPlainTextEdit(this);
    mText->setReadOnly(true);

    QPushButton* btnOK = new QPushButton("Accept");
    QPushButton* btnCancel = new QPushButton("Reject");

    QHBoxLayout* hlay = new QHBoxLayout();
    hlay->addWidget(btnOK);
    hlay->addWidget(btnCancel);

    QVBoxLayout* vlay = new QVBoxLayout();
    vlay->addWidget(mText);
    vlay->addLayout(hlay);

    setWindowTitle("Confirmation");
    setLayout(vlay);

    connect( btnOK, SIGNAL(clicked()), this, SLOT(accept()) );
    connect( btnCancel, SIGNAL(clicked()), this, SLOT(reject()) );
}

void CalibrationAcceptationDialog::setData(
    StereoRigCalibrationPtr calib,
    CalibrationResiduals& residuals)
{
    const QJsonDocument doc(calib->toJson().toObject());

    std::stringstream s;
    s << "Left calibration residuals = " << residuals.left_camera_calibration << std::endl;
    s << "Right calibration residuals = " << residuals.right_camera_calibration << std::endl;
    s << "Stereo calibration residuals = " << residuals.stereo_calibration << std::endl;
    s << std::endl;
    s << doc.toJson().data();

    mText->setPlainText(s.str().c_str());
}

