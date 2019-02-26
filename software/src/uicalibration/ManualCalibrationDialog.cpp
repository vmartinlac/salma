#include <QActionGroup>
#include <QSplitter>
#include <QMessageBox>
#include <QAction>
#include <QStatusBar>
#include <QToolBar>
#include <QVBoxLayout>
#include <opencv2/calib3d.hpp>
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
    QAction* aModePhotometric = tb->addAction("Photometric");
    tb->addSeparator();
    QAction* aPropagate = tb->addAction("Propagate");
    QAction* aCorner = tb->addAction("Corner");
    QAction* aConnection = tb->addAction("Connection");
    QAction* aClear = tb->addAction("Clear");
    QAction* aAutoDetect = tb->addAction("AutoDetect");
    tb->addSeparator();
    QAction* aHome = tb->addAction("Home");
    QAction* aCancel = tb->addAction("Cancel");
    QAction* aDone = tb->addAction("Submit");

    QActionGroup* grp_mode = new QActionGroup(this);
    grp_mode->addAction(aModeLeft);
    grp_mode->addAction(aModeRight);
    grp_mode->addAction(aModeStereo);
    grp_mode->addAction(aModePhotometric);

    aModeLeft->setCheckable(true);
    aModeRight->setCheckable(true);
    aModeStereo->setCheckable(true);
    aModePhotometric->setCheckable(true);

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

    mDataFrameList = new QListWidget();
    mDataFrameList->addItem("Frame 1");

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

    connect(aModeLeft, SIGNAL(triggered()), this, SLOT(setModeToLeft()));
    connect(aModeRight, SIGNAL(triggered()), this, SLOT(setModeToRight()));
    connect(aModeStereo, SIGNAL(triggered()), this, SLOT(setModeToStereo()));
    connect(aModePhotometric, SIGNAL(triggered()), this, SLOT(setModeToPhotometric()));

    connect(mSlider, SIGNAL(valueChanged(int)), this, SLOT(setFrame(int)));
    connect(aCancel, SIGNAL(triggered()), this, SLOT(reject()));
    connect(aDone, SIGNAL(triggered()), this, SLOT(accept()));
    connect(aHome, SIGNAL(triggered()), mView, SLOT(home()));
    connect(aAutoDetect, SIGNAL(triggered()), mView, SLOT(autoDetect()));
    connect(aClear, SIGNAL(triggered()), mView, SLOT(clear()));

    connect(mView, SIGNAL(listOfFramesWithDataChanged()), this, SLOT(updateListOfFramesWithData()));

    connect(mDataFrameList, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(frameWithDataClicked(QListWidgetItem*)));

    QMetaObject::invokeMethod(this, "setFrame", Q_ARG(int,0));
}

void ManualCalibrationDialog::setFrame(int frame)
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

void ManualCalibrationDialog::accept()
{
    CameraCalibrationDataPtr calib(new CameraCalibrationData());
    bool ok = true;
    const char* err = "";
    double projection_err = 0.0;

    std::vector< std::vector<cv::Point2f> > image_points;
    std::vector< std::vector<cv::Point3f> > object_points;

    // Retrieve calibration data.

    if(ok)
    {
        calib->name = mParameters->name.toStdString();

        ok = mView->getCalibrationData(
            image_points,
            object_points,
            calib->image_size);

        err = "Internal error";
    }

    // Temove views on which there are not enough points.
    // Then check that we have enough points for the calibration.

    if(ok)
    {
        if(image_points.size() != object_points.size()) throw std::runtime_error("internal error");

        {
            int i = 0;

            while(i<image_points.size())
            {
                if(image_points[i].size() != object_points[i].size()) throw std::runtime_error("internal error");

                if(image_points[i].size() >= 3*3)
                {
                    i++;
                }
                else
                {
                    image_points[i] = std::move(image_points.back());
                    object_points[i] = std::move(object_points.back());

                    image_points.pop_back();
                    object_points.pop_back();
                }
            }
        }

        ok = (image_points.size() >= 3);
        err = "Not enough points or orientations of the target!";
    }

    // Call OpenCV for the calibration.

    if(ok)
    {
        cv::Mat rvecs;
        cv::Mat tvecs;

        projection_err = cv::calibrateCamera(
            object_points,
            image_points,
            calib->image_size, // TODO: set this beforehand!
            calib->calibration_matrix,
            calib->distortion_coefficients,
            rvecs,
            tvecs);
    }

    // Save calibration into the project.

    if(ok)
    {
        int camera_id;
        ok = mProject->saveCamera(calib, camera_id);
        err = "Could not save camera!";
    }

    // Tell outcome to the user.

    if(ok)
    {
        QMessageBox::information(this, "Success", "Successful calibration! Reprojection error is " + QString::number(projection_err));
        QDialog::accept();
    }
    else
    {
        QMessageBox::critical(this, "Error", err);
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

void ManualCalibrationDialog::setModeToPhotometric()
{
    mView->setMode(ManualCalibrationView::MODE_PHOTOMETRIC);
    //updateListOfFramesWithData();
}

void ManualCalibrationDialog::updateListOfFramesWithData()
{
    std::vector<int> list;
    mView->enumerateFramesWithData(list);

    mDataFrameList->clear();

    for(int id : list)
    {
        QListWidgetItem* item = new QListWidgetItem(QString("Frame %1").arg(id));
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
        ok = (0 <= id && id < mParameters->recording->num_frames);
    }

    if(ok)
    {
        mView->setFrame(id);
    }
    else
    {
        QMessageBox::critical(this, "Error", "Incorrect frame!");
    }
}

