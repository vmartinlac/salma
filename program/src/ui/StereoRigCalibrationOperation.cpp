#include <iostream>
#include <future>
#include <thread>
#include <sstream>
#include <fstream>
#include <QThread>
#include <QTime>
#include <sophus/average.hpp>
#include <sophus/interpolate.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Eigen>
#include "Image.h"
#include "Tracker.h"
#include "StereoRigCalibrationOperation.h"
#include "StereoRigCalibrationData.h"

StereoRigCalibrationOperation::StereoRigCalibrationOperation()
{
    mNumberOfPosesForCalibration = 20;
    mMillisecondsOfTemporisation = 700;
    mTargetCellLength = 1.0;
}

StereoRigCalibrationOperation::~StereoRigCalibrationOperation()
{
}

bool StereoRigCalibrationOperation::before()
{
    mFrameCount = 0;
    mClock.start();
    mPoses.clear();

    mLeftTracker.setUnitLength(mTargetCellLength);
    mRightTracker.setUnitLength(mTargetCellLength);

    bool ok = true;

    if(ok)
    {
        // check that output file can be open.

        std::ofstream outputfile(mOutputPath.c_str(), std::ofstream::out);
        if(outputfile.is_open())
        {
            outputfile.close();
        }
        else
        {
            mStatsPort->beginWrite();
            mStatsPort->data().text = "Could not open output file!";
            mStatsPort->endWrite();
            ok = false;
        }
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

bool StereoRigCalibrationOperation::step()
{
    Sophus::SE3d left_camera_to_target;
    Sophus::SE3d right_camera_to_target;

    bool ret = true;

    bool go_on = true;

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

        if( concat.isValid() == false ) throw std::runtime_error("some unexpected error");

        mVideoPort->beginWrite();
        mVideoPort->data().image = concat.getFrame();
        mVideoPort->endWrite();

        mFrameCount++;
    }

    if( go_on )
    {
        go_on = image.isValid() && ( mPoses.empty() || mClock.elapsed() > mMillisecondsOfTemporisation );
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
        go_on = computePose(mLeftTracker, mLeftCalibrationData, left_camera_to_target);
    }

    if( go_on )
    {
        go_on = computePose(mRightTracker, mRightCalibrationData, right_camera_to_target);
    }

    if( go_on )
    {
        Sophus::SE3d right_camera_to_left_camera = left_camera_to_target.inverse() * right_camera_to_target;

        mPoses.push_back( right_camera_to_left_camera );

        mClock.start();
    }


    if( go_on && mPoses.size() >= mNumberOfPosesForCalibration )
    {
        calibrate();
        ret = false;
    }
    else
    {
        writeOutputText();
    }

    return ret;
}

void StereoRigCalibrationOperation::after()
{
    if(mCamera)
    {
        mCamera->close();
    }
}

void StereoRigCalibrationOperation::writeOutputText()
{
    std::stringstream s;
    s << "Frame count: " << mFrameCount << std::endl;
    s << "Number of computed poses: " << mPoses.size() << std::endl;
    s << "Number of poses left: " << mNumberOfPosesForCalibration - mPoses.size() << std::endl;
    s << std::endl;
    s << "Video input: " << mCamera->getHumanName() << std::endl;
    s << "Target cell length: " << mTargetCellLength << std::endl;
    s << "Output file: " << mOutputPath << std::endl;

    mStatsPort->beginWrite();
    mStatsPort->data().text = s.str().c_str();
    mStatsPort->endWrite();
}

bool StereoRigCalibrationOperation::computePose(target::Tracker& tracker, CameraCalibrationData& calibration, Sophus::SE3d& camera_to_target)
{
    cv::Mat pnp_rodrigues;
    cv::Mat pnp_translation;

    bool ok = true;

    if( ok )
    {
        ok = ( tracker.objectPoints().size() >= 15 );
    }

    if( ok )
    {
        ok = cv::solvePnP(
            tracker.objectPoints(),
            tracker.imagePoints(),
            calibration.calibration_matrix,
            calibration.distortion_coefficients,
            pnp_rodrigues,
            pnp_translation,
            false,
            cv::SOLVEPNP_ITERATIVE );
    }

    if(ok)
    {
        convertPoseFromOpenCVToSophus(pnp_rodrigues, pnp_translation, camera_to_target);
    }

    return ok;
}

void StereoRigCalibrationOperation::calibrate()
{
    Sophus::optional< Sophus::SE3d > right_camera_to_left_camera;
    double xdev = 0.0;
    double ydev = 0.0;
    double zdev = 0.0;
    double thetadev = 0.0;
    StereoRigCalibrationData calibration;
    const char* error_message = "";
    bool ok = true;

    if(ok)
    {
        ok = (mPoses.size() > 1);
    }

    if(ok)
    {
        right_camera_to_left_camera = Sophus::average( mPoses );
        ok = bool(right_camera_to_left_camera);
        error_message = "Could not average calibration data.";
    }

    if(ok)
    {
        xdev = 0.0;
        ydev = 0.0;
        zdev = 0.0;
        thetadev = 0.0;

        for( Sophus::SE3d& p : mPoses)
        {
            const double dx = p.translation().x() - right_camera_to_left_camera->translation().x();
            const double dy = p.translation().y() - right_camera_to_left_camera->translation().y();
            const double dz = p.translation().z() - right_camera_to_left_camera->translation().z();
            const double dtheta = p.unit_quaternion().angularDistance( right_camera_to_left_camera->unit_quaternion() );

            xdev += dx*dx;
            ydev += dy*dy;
            zdev += dz*dz;
            thetadev += dtheta*dtheta;
        }

        xdev = std::sqrt( xdev/double(mPoses.size()) );
        ydev = std::sqrt( ydev/double(mPoses.size()) );
        zdev = std::sqrt( zdev/double(mPoses.size()) );
        thetadev = std::sqrt( thetadev/double(mPoses.size()) );
    }

    if(ok)
    {
        //Sophus::interpolate( Sophus::SE3d(), right_camera_to_world, 0.5 );
        // TODO: world should be half-way between left and right cameras.

        calibration.left_camera_to_world = Sophus::SE3d();
        calibration.right_camera_to_world = *right_camera_to_left_camera;
    }
    
    if(ok)
    {
        ok = calibration.saveToFile(mOutputPath);
        error_message = "Error while saving calibration data to file.";
    }

    if(ok)
    {
        std::stringstream s;

        s << "Left camera to world transformation:" << std::endl;
        s << "left_x = " << calibration.left_camera_to_world.translation().x() << std::endl;
        s << "left_y = " << calibration.left_camera_to_world.translation().y() << std::endl;
        s << "left_z = " << calibration.left_camera_to_world.translation().z() << std::endl;
        s << "left_qx = " << calibration.left_camera_to_world.unit_quaternion().x() << std::endl;
        s << "left_qy = " << calibration.left_camera_to_world.unit_quaternion().y() << std::endl;
        s << "left_qz = " << calibration.left_camera_to_world.unit_quaternion().z() << std::endl;
        s << "left_qw = " << calibration.left_camera_to_world.unit_quaternion().w() << std::endl;
        s << std::endl;

        s << "Right camera to world transformation:" << std::endl;
        s << "right_x = " << calibration.right_camera_to_world.translation().x() << std::endl;
        s << "right_y = " << calibration.right_camera_to_world.translation().y() << std::endl;
        s << "right_z = " << calibration.right_camera_to_world.translation().z() << std::endl;
        s << "right_qx = " << calibration.right_camera_to_world.unit_quaternion().x() << std::endl;
        s << "right_qy = " << calibration.right_camera_to_world.unit_quaternion().y() << std::endl;
        s << "right_qz = " << calibration.right_camera_to_world.unit_quaternion().z() << std::endl;
        s << "right_qw = " << calibration.right_camera_to_world.unit_quaternion().w() << std::endl;
        s << std::endl;

        s << "x std-dev = " << xdev << std::endl;
        s << "y std-dev = " << ydev << std::endl;
        s << "z std-dev = " << zdev << std::endl;
        s << "θ std-dev = " << thetadev << std::endl;
        s << std::endl;

        mStatsPort->beginWrite();
        mStatsPort->data().text = s.str().c_str();
        mStatsPort->endWrite();
    }
    else
    {
        mStatsPort->beginWrite();
        mStatsPort->data().text = error_message;
        mStatsPort->endWrite();
    }
}

void StereoRigCalibrationOperation::convertPoseFromOpenCVToSophus(
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

