#include <QActionGroup>
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
    QAction* aCorner = tb->addAction("Corner");
    QAction* aConnection = tb->addAction("Connection");
    QAction* aClear = tb->addAction("Clear");
    QAction* aHome = tb->addAction("Home");
    tb->addSeparator();
    QAction* aCancel = tb->addAction("Cancel");
    QAction* aDone = tb->addAction("Done");

    aCorner->setCheckable(true);
    aConnection->setCheckable(true);
    aCorner->setChecked(true);

    QActionGroup* grp = new QActionGroup(this);
    grp->addAction(aCorner);
    grp->addAction(aConnection);

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

    connect(aCorner, SIGNAL(triggered()), mView, SLOT(setModeToCorner()));
    connect(aConnection, SIGNAL(triggered()), mView, SLOT(setModeToConnection()));
    connect(mSlider, SIGNAL(valueChanged(int)), this, SLOT(setFrame(int)));
    connect(aCancel, SIGNAL(triggered()), this, SLOT(reject()));
    connect(aDone, SIGNAL(triggered()), this, SLOT(accept()));
    connect(aHome, SIGNAL(triggered()), mView, SLOT(home()));
    connect(aClear, SIGNAL(triggered()), mView, SLOT(clear()));

    QMetaObject::invokeMethod(this, "setFrame", Q_ARG(int,0));
}

void ManualCameraCalibrationDialog::setFrame(int frame)
{
    if(0 <= frame && frame < mParameters->recording->num_frames)
    {
        mLabelFrame->setText( QString("Frame %1/%2").arg(frame+1).arg(mParameters->recording->num_frames) );
    }
    else
    {
        mLabelFrame->setText("N/A");
    }

    mView->setFrame(frame);
}

void ManualCameraCalibrationDialog::accept()
{
    // TODO
    QDialog::accept();
}

