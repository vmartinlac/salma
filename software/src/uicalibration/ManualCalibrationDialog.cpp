#include <QActionGroup>
#include <QSplitter>
#include <QMessageBox>
#include <QAction>
#include <QStatusBar>
#include <QToolBar>
#include <QVBoxLayout>
#include <opencv2/calib3d.hpp>
#include "CalibrationAcceptationDialog.h"
#include "ManualCalibrationView.h"
#include "ManualCalibrationDialog.h"

ManualCalibrationDialog::ManualCalibrationDialog(
    Project* project,
    ManualCalibrationParametersPtr params,
    QWidget* parent)
{
    mProject = project;
    mParameters = params;

    QToolBar* tb = new QToolBar();
    QAction* aModeLeft = tb->addAction("Left");
    QAction* aModeRight = tb->addAction("Right");
    QAction* aModeStereo = tb->addAction("Stereo");
    tb->addSeparator();
    QAction* aClear = tb->addAction("Clear");
    QAction* aTake = tb->addAction("Take");
    tb->addSeparator();
    QAction* aHome = tb->addAction("Home");
    QAction* aCancel = tb->addAction("Cancel");
    QAction* aDone = tb->addAction("Submit");

    QActionGroup* grp_mode = new QActionGroup(this);
    grp_mode->addAction(aModeLeft);
    grp_mode->addAction(aModeRight);
    grp_mode->addAction(aModeStereo);

    aModeLeft->setCheckable(true);
    aModeRight->setCheckable(true);
    aModeStereo->setCheckable(true);

    mSlider = new QSlider;
    mSlider->setOrientation(Qt::Horizontal);
    mSlider->setMinimum(0);
    mSlider->setMaximum(params->recording->num_frames());

    mLabelFrame = new QLabel("N/A");

    QStatusBar* sb = new QStatusBar();
    sb->addPermanentWidget(mLabelFrame);

    mDataFrameList = new QListWidget();

    mView = new ManualCalibrationView(mParameters, this);

    QSplitter* splitter = new QSplitter();
    splitter->setChildrenCollapsible(false);
    splitter->addWidget(mDataFrameList);
    splitter->addWidget(mView);
    splitter->setSizes({width()/4, 3*width()/4});

    QVBoxLayout* lay = new QVBoxLayout();
    lay->addWidget(tb, 0);
    lay->addWidget(mSlider, 0);
    lay->addWidget(splitter, 1);
    lay->addWidget(sb);

    setLayout(lay);
    setWindowTitle("Manual Camera Calibration");
    showMaximized();

    connect(aModeLeft, SIGNAL(triggered()), this, SLOT(setModeToLeft()));
    connect(aModeRight, SIGNAL(triggered()), this, SLOT(setModeToRight()));
    connect(aModeStereo, SIGNAL(triggered()), this, SLOT(setModeToStereo()));

    connect(mSlider, SIGNAL(valueChanged(int)), this, SLOT(setFrame(int)));
    connect(aCancel, SIGNAL(triggered()), this, SLOT(reject()));
    connect(aDone, SIGNAL(triggered()), this, SLOT(accept()));
    connect(aHome, SIGNAL(triggered()), mView, SLOT(home()));
    connect(aTake, SIGNAL(triggered()), mView, SLOT(doTake()));
    connect(aClear, SIGNAL(triggered()), mView, SLOT(doClear()));

    connect(mView, SIGNAL(listOfFramesWithDataChanged()), this, SLOT(updateListOfFramesWithData()));

    connect(mDataFrameList, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(frameWithDataClicked(QListWidgetItem*)));

    aModeLeft->setChecked(true);
    setModeToLeft();

    QMetaObject::invokeMethod(this, "setFrame", Qt::QueuedConnection, Q_ARG(int,0));
}

void ManualCalibrationDialog::setFrame(int frame)
{
    if(0 <= frame && frame < mParameters->recording->num_frames())
    {
        mLabelFrame->setText( QString("Frame %1/%2").arg(frame+1).arg(mParameters->recording->num_frames()) );
    }
    else
    {
        mLabelFrame->setText("N/A");
    }

    mView->setFrame(frame);
}

void ManualCalibrationDialog::accept()
{
    StereoRigCalibrationPtr calib;
    CalibrationResiduals residuals;

    const bool calib_ok = mView->doCalibrate(calib, residuals);

    if(calib_ok)
    {
        CalibrationAcceptationDialog* dlg = new CalibrationAcceptationDialog(this);

        dlg->setData( calib, residuals );

        const int dlg_ret = dlg->exec();

        delete dlg;

        if(dlg_ret == QDialog::Accepted)
        {
            int rig_id = -1;
            const bool db_ok = mProject->saveCalibration(calib, rig_id);

            if(db_ok)
            {
                QMessageBox::information(this, "Calibration", "Successful calibration!");
                QDialog::accept();
            }
            else
            {
                QMessageBox::critical(this, "Error", "Could not save calibration into database!");
            }
        }
    }
    else
    {
        QMessageBox::critical(this, "Error", "Calibration failed!");
    }
}

void ManualCalibrationDialog::setModeToLeft()
{
    mView->setMode(ManualCalibrationView::MODE_LEFT);
    //updateListOfFramesWithData();
}

void ManualCalibrationDialog::setModeToRight()
{
    mView->setMode(ManualCalibrationView::MODE_RIGHT);
    //updateListOfFramesWithData();
}

void ManualCalibrationDialog::setModeToStereo()
{
    mView->setMode(ManualCalibrationView::MODE_STEREO);
    //updateListOfFramesWithData();
}

void ManualCalibrationDialog::updateListOfFramesWithData()
{
    std::vector<int> list;
    mView->enumerateFramesWithData(list);

    mDataFrameList->clear();

    for(int id : list)
    {
        QListWidgetItem* item = new QListWidgetItem(QString("Frame %1").arg(id+1));
        item->setData(Qt::UserRole, id);
        mDataFrameList->addItem(item);
    }
}

void ManualCalibrationDialog::frameWithDataClicked(QListWidgetItem* item)
{
    bool ok = false;

    const int id = item->data(Qt::UserRole).toInt(&ok);


    if(ok)
    {
        ok = (0 <= id && id < mParameters->recording->num_frames());
    }

    if(ok)
    {
        //mView->setFrame(id);
        mSlider->setValue(id);
    }
    else
    {
        QMessageBox::critical(this, "Error", "Incorrect frame!");
    }
}

