#include <iostream>
#include <sstream>
#include <fstream>
#include <QThread>
#include <QTime>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include "Image.h"
#include "Tracker.h"
#include "CameraCalibrationOperation.h"
#include "CameraCalibrationData.h"

CameraCalibrationOperation::CameraCalibrationOperation()
{
    mRequestedSuccessfulFrameCount = 40;
    mMillisecondsTemporisation = 700;
}

CameraCalibrationOperation::~CameraCalibrationOperation()
{
}

bool CameraCalibrationOperation::before()
{
    mFrameCount = 0;
    mSuccessfulFrameCount = 0;
    mAttemptedFrameCount = 0;
    mObjectPoints.clear();
    mImagePoints.clear();
    mImageSize = cv::Size(-1, -1);
    mClock.start();

    bool ok = true;
    
    if(ok)
    {
        ok = bool(mCamera);
    }

    if(ok)
    {
        ok = mCamera->open();
    }

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
            ok = false;
            std::cout << "Could not open output file!" << std::endl; // TODO: put this message on the UI.
        }
    }

    return ok;
}

bool CameraCalibrationOperation::step()
{
    bool ret = true;

    bool can_calibrate = false;

    Image image;
    image.setValid(false);

    if( mCamera )
    {
        mCamera->read(image);
    }
    else
    {
        ret = false;
    }

    if( image.isValid() )
    {
        if( mObjectPoints.empty() || mClock.elapsed() > mMillisecondsTemporisation )
        {
            const int target_found = mTracker.track(image.refFrame(), false);

            if( target_found )
            {
                mImagePoints.push_back( mTracker.imagePoints() );
                mObjectPoints.push_back( mTracker.objectPoints() );

                mSuccessfulFrameCount++;

                mClock.start();

                mImageSize = image.refFrame().size();
                can_calibrate = (mObjectPoints.size() >= mRequestedSuccessfulFrameCount );
            }

            mAttemptedFrameCount++;
        }

        mFrameCount++;

        mVideoPort->beginWrite();
        mVideoPort->data().image = image.refFrame();
        mVideoPort->endWrite();

        writeOutputText();
    }

    if( can_calibrate )
    {
        CameraCalibrationData calibration;

        calibration.image_size = mImageSize;

        std::vector<cv::Mat> rotations;
        std::vector<cv::Mat> translations;

        const double err = cv::calibrateCamera(
            mObjectPoints,
            mImagePoints,
            mImageSize,
            calibration.calibration_matrix,
            calibration.distortion_coefficients,
            rotations,
            translations);
        
        const bool save_ret = calibration.saveToFile(mOutputPath);

        if(save_ret)
        {
            std::stringstream s;

            s << "Reprojection error: " << err << std::endl;
            s << std::endl;

            s << "Camera matrix:" << std::endl;
            s << calibration.calibration_matrix.at<double>(0,0) << ' ';
            s << calibration.calibration_matrix.at<double>(0,1) << ' ';
            s << calibration.calibration_matrix.at<double>(0,2) << std::endl;
            s << calibration.calibration_matrix.at<double>(1,0) << ' ';
            s << calibration.calibration_matrix.at<double>(1,1) << ' ';
            s << calibration.calibration_matrix.at<double>(1,2) << std::endl;
            s << calibration.calibration_matrix.at<double>(2,0) << ' ';
            s << calibration.calibration_matrix.at<double>(2,1) << ' ';
            s << calibration.calibration_matrix.at<double>(2,2) << std::endl;
            s << std::endl;

            s << "Distortion coefficients:" << std::endl;
            for(int i=0; i<calibration.distortion_coefficients.rows; i++)
            {
                s << calibration.distortion_coefficients.at<double>(i,0) << std::endl;
            }
            s << std::endl;

            mStatsPort->beginWrite();
            mStatsPort->data().text = s.str().c_str();
            mStatsPort->endWrite();
        }
        else
        {
            mStatsPort->beginWrite();
            mStatsPort->data().text = "Could not save calibration data to file!";
            mStatsPort->endWrite();
        }

        ret = false;
    }

    QThread::msleep(5);

    return ret;
}

void CameraCalibrationOperation::after()
{
    if(mCamera)
    {
        mCamera->close();
    }
}

void CameraCalibrationOperation::writeOutputText()
{
    std::stringstream s;
    s << "Frame count: " << mFrameCount << std::endl;
    s << "Attempted frame count: " << mAttemptedFrameCount << std::endl;
    s << "Successful frame count: " << mSuccessfulFrameCount << std::endl;
    s << "Successful frames left: " << mRequestedSuccessfulFrameCount - mSuccessfulFrameCount << std::endl;
    s << std::endl;
    s << "Camera name: " << mCamera->getHumanName() << std::endl;
    s << "Output file: " << mOutputPath << std::endl;

    mStatsPort->beginWrite();
    mStatsPort->data().text = s.str().c_str();
    mStatsPort->endWrite();
}

