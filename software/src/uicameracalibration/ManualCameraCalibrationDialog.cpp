#include <QAction>
#include <QStatusBar>
#include <QToolBar>
#include <QVBoxLayout>
#include "ManualCameraCalibrationView.h"
#include "ManualCameraCalibrationDialog.h"

ManualCameraCalibrationDialog::ManualCameraCalibrationDialog(
    Project* project,
    ManualCameraCalibrationParametersPtr params,
    QWidget* parent)
{
    mProject = project;
    mParameters = params;

    QToolBar* tb = new QToolBar();
    QAction* aBegin = tb->addAction("Begin");
    QAction* aEnd = tb->addAction("End");
    tb->addSeparator();
    QAction* aPoint = tb->addAction("Point");
    QAction* aConnection = tb->addAction("Connection");
    QAction* aClear = tb->addAction("Clear");
    QAction* aHome = tb->addAction("Home");
    tb->addSeparator();
    QAction* aCancel = tb->addAction("Cancel");
    QAction* aDone = tb->addAction("Done");

    mSlider = new QSlider;
    mSlider->setOrientation(Qt::Horizontal);
    mSlider->setMinimum(0);
    mSlider->setMaximum(params->recording->num_frames);

    mLabelFrame = new QLabel("N/A");

    QStatusBar* sb = new QStatusBar();
    sb->addPermanentWidget(mLabelFrame);

    mView = new ManualCameraCalibrationView(mParameters, this);

    QVBoxLayout* lay = new QVBoxLayout();
    lay->addWidget(tb, 0);
    lay->addWidget(mSlider, 0);
    //lay->addWidget(new QScrollArea(), 1);
    lay->addWidget(mView, 1);
    lay->addWidget(sb);

    setLayout(lay);
    setWindowTitle("Manual Camera Calibration");

    connect(mSlider, SIGNAL(valueChanged(int)), mView, SLOT(setFrame(int)));
    connect(aCancel, SIGNAL(triggered()), this, SLOT(reject()));
    connect(aDone, SIGNAL(triggered()), this, SLOT(accept()));
    connect(aHome, SIGNAL(triggered()), mView, SLOT(home()));

    QMetaObject::invokeMethod(mView, "setFrame", Q_ARG(int,0));
}

void ManualCameraCalibrationDialog::accept()
{
    // TODO
    QDialog::accept();
}

