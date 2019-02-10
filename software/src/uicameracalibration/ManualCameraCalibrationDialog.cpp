#include <QActionGroup>
#include <QMessageBox>
#include <QAction>
#include <QStatusBar>
#include <QToolBar>
#include <QVBoxLayout>
#include <opencv2/calib3d.hpp>
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
    QAction* aAutoDetect = tb->addAction("AutoDetect");
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
    connect(aAutoDetect, SIGNAL(triggered()), mView, SLOT(autoDetect()));
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

