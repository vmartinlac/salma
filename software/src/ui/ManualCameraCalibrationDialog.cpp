#include <QAction>
#include <QStatusBar>
#include <QToolBar>
#include <QVBoxLayout>
#include <QScrollArea>
#include "ManualCameraCalibrationDialog.h"

ManualCameraCalibrationDialog::ManualCameraCalibrationDialog(
    Project* project,
    ManualCameraCalibrationParametersPtr params,
    QWidget* parent)
{
    mProject = project;
    mParameters = params;

    QToolBar* tb = new QToolBar();
    tb->addAction("Point");
    tb->addAction("Connection");
    tb->addAction("Clear");
    tb->addAction("Done");

    mSlider = new QSlider;
    mSlider->setOrientation(Qt::Horizontal);

    mLabelFrame = new QLabel("N/A");

    QStatusBar* sb = new QStatusBar();
    sb->addPermanentWidget(mLabelFrame);

    QVBoxLayout* lay = new QVBoxLayout();
    lay->addWidget(tb, 0);
    lay->addWidget(mSlider, 0);
    lay->addWidget(new QScrollArea(), 1);
    lay->addWidget(sb);

    setLayout(lay);
    setWindowTitle("Manual Camera Calibration");
}

