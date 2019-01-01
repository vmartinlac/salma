#include <iostream>
#include <future>
#include <thread>
#include <sstream>
#include <fstream>
#include <QMessageBox>
#include <QThread>
#include <QTime>
#include <sophus/average.hpp>
#include <sophus/interpolate.hpp>
#include <Eigen/Eigen>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/eigen.hpp>
#include "Image.h"
#include "Tracker.h"
#include "RigCalibrationOperation.h"
#include "StereoRigCalibrationData.h"
#include "Project.h"

RigCalibrationOperation::RigCalibrationOperation()
{
    mNumberOfPosesForCalibration = 20;
    mMillisecondsOfTemporisation = 700;
    mTargetCellLength = 1.0;
}

RigCalibrationOperation::~RigCalibrationOperation()
{
}

bool RigCalibrationOperation::before()
{
    mFrameCount = 0;
    mClock.start();

    mLeftTracker.setUnitLength(mTargetCellLength);
    mRightTracker.setUnitLength(mTargetCellLength);

    mLeftImagePoints.clear();
    mRightImagePoints.clear();
    mObjectPoints.clear();

    bool ok = true;

    if(ok)
    {
        ok = bool(mLeftCalibrationData) && bool(mRightCalibrationData);
    }

    if(ok)
    {
        ok = (mLeftCalibrationData->id >= 0 && mRightCalibrationData->id >= 0);
    }
    
    if(ok)
    {
        ok = bool(mCamera) && (mCamera->getNumberOfCameras() == 2);
    }

    if(ok)
    {
        ok = mCamera->open();
    }

    if(ok)
    {
        mCamera->trigger();
        mTriggerClock.start();
    }

    return ok;
}

bool RigCalibrationOperation::step()
{
    bool ret = true;

    bool go_on = true;

    std::vector<cv::Point2f> new_left_image_points;
    std::vector<cv::Point2f> new_right_image_points;
    std::vector<cv::Point3f> new_object_points;

    Image image;
    image.setInvalid();

    if(go_on)
    {
        if( mCamera )
        {
            mCamera->read(image);
        }
        else
        {
            ret = false;
            go_on = false;
        }
    }

    if(image.isValid() || mTriggerClock.elapsed() > 500)
    {
        mCamera->trigger();
        mTriggerClock.start();
    }

    if( image.isValid() )
    {
        Image concat;
        image.concatenate(concat);

        if( concat.isValid() )
        {
            videoPort()->beginWrite();
            videoPort()->data().image = concat.getFrame();
            videoPort()->endWrite();

            mFrameCount++;
        }
        else
        {
            std::cerr << "Error while concatenating images!" << std::endl;
            go_on = false;
        }
    }

    if( go_on )
    {
        go_on = image.isValid() && ( mLeftImagePoints.empty() || mClock.elapsed() > mMillisecondsOfTemporisation );
    }

    if( go_on )
    {
        std::future<bool> left_target_found = std::async( std::launch::async, [this, &image] () -> bool
        {
            return mLeftTracker.track(image.getFrame(0), true);
        });

        std::future<bool> right_target_found = std::async( std::launch::async, [this, &image] () -> bool
        {
            return mRightTracker.track(image.getFrame(1), true);
        });

        const bool left_one = left_target_found.get();
        const bool right_one = right_target_found.get();
        go_on = left_one && right_one;
    }

    if( go_on )
    {
        for(int i=0; i<mLeftTracker.pointIds().size(); i++)
        {
            const int id = mLeftTracker.pointIds()[i];

            int j = 0;
            while( j<mRightTracker.pointIds().size() && mRightTracker.pointIds()[j] != id )
            {
                j++;
            }

            if( j < mRightTracker.pointIds().size() )
            {
                if( mRightTracker.pointIds()[j] != id ) throw std::logic_error("internal error");

                new_left_image_points.push_back(mLeftTracker.imagePoints()[i]);
                new_right_image_points.push_back(mRightTracker.imagePoints()[j]);
                new_object_points.push_back(mLeftTracker.objectPoints()[i]);
            }
        }

        go_on = ( new_left_image_points.size() > 30 );
    }

    if( go_on )
    {
        mLeftImagePoints.push_back( std::move(new_left_image_points) );
        mRightImagePoints.push_back( std::move(new_right_image_points) );
        mObjectPoints.push_back( std::move(new_object_points) );

        mClock.start();
    }

    /*
    if( go_on )
    {
        auto plot_points = [this] ( cv::Mat image, const CameraCalibrationData& calibration, const Sophus::SE3d& camera_to_target, const char* prefix )
        {
            std::vector<cv::Point3d> arr;

            for(int i=-4; i<=4; i++)
            {
                for(int j=-4; j<=4; j++)
                {
                    Eigen::Vector3d point3d;
                    point3d.x() = mTargetCellLength*double(i);
                    point3d.y() = mTargetCellLength*double(j);
                    point3d.z() = 0.0;

                    Eigen::Vector3d P = (camera_to_target.inverse() * point3d);

                    arr.push_back( cv::Point3d( P.x(), P.y(), P.z() ) );
                }
            }

            std::vector<cv::Point2d> pts(arr.size());

            cv::projectPoints(
                arr,
                cv::Mat::zeros(3, 1, CV_64F),
                cv::Mat::zeros(3, 1, CV_64F),
                calibration.calibration_matrix,
                calibration.distortion_coefficients,
                pts);

            for( cv::Point2f pt : pts )
            {
                cv::circle(image, pt, 4, cv::Scalar(0, 255, 0), -1 );;
            }

            static int i = 0;
            cv::imwrite("/home/victor/depotoire/rien_"+std::string(prefix)+std::to_string(i)+".png", image);
            i++;
        };

        plot_points( image.getFrame(0), mLeftCalibrationData, left_camera_to_target, "left");
        plot_points( image.getFrame(1), mRightCalibrationData, right_camera_to_target, "right");
    }
    */

    if( go_on && mLeftImagePoints.size() >= mNumberOfPosesForCalibration )
    {
        statsPort()->beginWrite();
        statsPort()->data().text = "Computing calibration data ...";
        statsPort()->endWrite();

        calibrate();
        ret = false;
    }
    else
    {
        writeOutputText();
    }

    return ret;
}

void RigCalibrationOperation::after()
{
    if(mCamera)
    {
        mCamera->close();
    }
}

void RigCalibrationOperation::writeOutputText()
{
    std::stringstream s;
    s << "Frame count: " << mFrameCount << std::endl;
    s << "Number of computed poses: " << mLeftImagePoints.size() << std::endl;
    s << "Number of poses left: " << mNumberOfPosesForCalibration - mLeftImagePoints.size() << std::endl;
    s << std::endl;
    s << "Video input: " << mCamera->getHumanName() << std::endl;
    s << "Target cell length: " << mTargetCellLength << std::endl;
    s << "Rig calibration name: " << mCalibrationName << std::endl;

    statsPort()->beginWrite();
    statsPort()->data().text = s.str().c_str();
    statsPort()->endWrite();
}

void RigCalibrationOperation::calibrate()
{
    bool ok = true;
    const char* error_message = "";

    StereoRigCalibrationDataPtr calibration(new StereoRigCalibrationData());

    cv::Mat R;
    cv::Mat T;
    cv::Mat E;
    cv::Mat F;
    double err = 0.0;

    if(ok)
    {
        cv::TermCriteria term(
            cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
            30, 1.0e-6);

        err = cv::stereoCalibrate(
            mObjectPoints,
            mLeftImagePoints, mRightImagePoints,
            mLeftCalibrationData->calibration_matrix, mLeftCalibrationData->distortion_coefficients,
            mRightCalibrationData->calibration_matrix, mRightCalibrationData->distortion_coefficients,
            mLeftCalibrationData->image_size, R, T, E, F,
            cv::CALIB_FIX_INTRINSIC,
            term);
    }

    if(ok)
    {
        Eigen::Matrix3d Rbis;
        Eigen::Vector3d Tbis;

        cv::cv2eigen<double,3,3>(R, Rbis);
        cv::cv2eigen<double,3,1>(T, Tbis);

        calibration->name = mCalibrationName;

        calibration->cameras[0].camera_to_rig = Sophus::SE3d();

        calibration->cameras[1].camera_to_rig.setRotationMatrix(Rbis.transpose());
        calibration->cameras[1].camera_to_rig.translation() = -Rbis.transpose() * Tbis;

        calibration->cameras[0].calibration = mLeftCalibrationData;
        calibration->cameras[1].calibration = mRightCalibrationData;

        //cv::cv2eigen<double,3,3>(F, calibration->fundamental_matrix);
        //cv::cv2eigen<double,3,3>(E, calibration->essential_matrix);
    }
    
    if(ok)
    {
        mResult = calibration;
    }

    if(ok)
    {
        std::stringstream s;

        s << "Residual error = " << err << std::endl;
        s << std::endl;

        s << "Left camera to rig transformation:" << std::endl;
        s << "left_x = " << calibration->cameras[0].camera_to_rig.translation().x() << std::endl;
        s << "left_y = " << calibration->cameras[0].camera_to_rig.translation().y() << std::endl;
        s << "left_z = " << calibration->cameras[0].camera_to_rig.translation().z() << std::endl;
        s << "left_qx = " << calibration->cameras[0].camera_to_rig.unit_quaternion().x() << std::endl;
        s << "left_qy = " << calibration->cameras[0].camera_to_rig.unit_quaternion().y() << std::endl;
        s << "left_qz = " << calibration->cameras[0].camera_to_rig.unit_quaternion().z() << std::endl;
        s << "left_qw = " << calibration->cameras[0].camera_to_rig.unit_quaternion().w() << std::endl;
        s << std::endl;

        s << "Right camera to rig transformation:" << std::endl;
        s << "right_x = " << calibration->cameras[1].camera_to_rig.translation().x() << std::endl;
        s << "right_y = " << calibration->cameras[1].camera_to_rig.translation().y() << std::endl;
        s << "right_z = " << calibration->cameras[1].camera_to_rig.translation().z() << std::endl;
        s << "right_qx = " << calibration->cameras[1].camera_to_rig.unit_quaternion().x() << std::endl;
        s << "right_qy = " << calibration->cameras[1].camera_to_rig.unit_quaternion().y() << std::endl;
        s << "right_qz = " << calibration->cameras[1].camera_to_rig.unit_quaternion().z() << std::endl;
        s << "right_qw = " << calibration->cameras[1].camera_to_rig.unit_quaternion().w() << std::endl;
        s << std::endl;

        statsPort()->beginWrite();
        statsPort()->data().text = s.str().c_str();
        statsPort()->endWrite();
    }
    else
    {
        statsPort()->beginWrite();
        statsPort()->data().text = error_message;
        statsPort()->endWrite();
    }
}

const char* RigCalibrationOperation::getName()
{
    return "Rig Calibration";
}

void RigCalibrationOperation::uiafter(QWidget* parent, Project* proj)
{
    if(mResult)
    {
        bool ok = true;
        int rig_id = -1;

        if(ok)
        {
            ok = proj->transaction();
        }

        if(ok)
        {
            ok = proj->saveRig(mResult, rig_id);
        }

        if(ok)
        {
            ok = proj->commit();
        }
        else
        {
            proj->rollback();
        }

        if(ok)
        {
            QMessageBox::information(parent, "Success", "Calibration done!");
        }
        else
        {
            QMessageBox::critical(parent, "Error", "Failed to save calibration to database!");
        }
    }
    else
    {
        QMessageBox::critical(parent, "Error", "Calibration failed!");
    }
}

/*
void RigCalibrationOperation::convertPoseFromOpenCVToSophus(
    const cv::Mat& rodrigues,
    const cv::Mat& t,
    Sophus::SE3d& camera_to_object)
{
    Eigen::Vector3d rodrigues_eigen;

    if( rodrigues.type() == CV_32F )
    {
        rodrigues_eigen.x() = rodrigues.at<float>(0);
        rodrigues_eigen.y() = rodrigues.at<float>(1);
        rodrigues_eigen.z() = rodrigues.at<float>(2);
    }
    else if( rodrigues.type() == CV_64F )
    {
        rodrigues_eigen.x() = rodrigues.at<double>(0);
        rodrigues_eigen.y() = rodrigues.at<double>(1);
        rodrigues_eigen.z() = rodrigues.at<double>(2);
    }

    Eigen::Vector3d t_eigen;
    if( t.type() == CV_32F )
    {
        t_eigen.x() = t.at<float>(0);
        t_eigen.y() = t.at<float>(1);
        t_eigen.z() = t.at<float>(2);
    }
    else if( t.type() == CV_64F )
    {
        t_eigen.x() = t.at<double>(0);
        t_eigen.y() = t.at<double>(1);
        t_eigen.z() = t.at<double>(2);
    }

    const double norm = rodrigues_eigen.norm();

    Eigen::Quaterniond attitude;
    Eigen::Vector3d position;

    if( norm > 1.0e-8 )
    {
        attitude.vec() = -sin(0.5*norm) * rodrigues_eigen / norm;
        attitude.w() = cos(0.5*norm);
    }
    else
    {
        attitude.setIdentity();
    }

    position = -( attitude * t_eigen );

    camera_to_object.translation() = position;
    camera_to_object.setQuaternion(attitude);
}
*/

