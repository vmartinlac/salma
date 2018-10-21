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
        mCamera->trigger();
    }

    return ok;
}

bool CameraCalibrationOperation::step()
{
    bool ret = true;

    bool can_calibrate = false;

    Image image;
    image.setInvalid();

    if( mCamera )
    {
        mCamera->read(image);
        mCamera->trigger();
    }
    else
    {
        ret = false;
    }

    if( image.isValid() )
    {
        if( mObjectPoints.empty() || mClock.elapsed() > mMillisecondsTemporisation )
        {
            const bool target_found = mTracker.track(image.getFrame(), false);

            if( target_found )
            {
                mImagePoints.push_back( mTracker.imagePoints() );
                mObjectPoints.push_back( mTracker.objectPoints() );

                mSuccessfulFrameCount++;

                mClock.start();

                mImageSize = image.getFrame().size();
                can_calibrate = (mObjectPoints.size() >= mRequestedSuccessfulFrameCount );
            }

            mAttemptedFrameCount++;
        }

        mFrameCount++;

        mVideoPort->beginWrite();
        mVideoPort->data().image = image.getFrame();
        mVideoPort->endWrite();

        writeOutputText();
    }

    if( can_calibrate )
    {
        CameraCalibrationData calibration;

        calibration.image_size = mImageSize;

        std::vector<cv::Mat> rotations;
        std::vector<cv::Mat> translations;

        mStatsPort->beginWrite();
        mStatsPort->data().text = "Computing calibration data ...";
        mStatsPort->endWrite();

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

            auto write_mat = [&s] (const cv::Mat& m, bool matrixlayout)
            {
                for(int i=0; i<m.rows; i++)
                {
                    for(int j=0; j<m.cols; j++)
                    {
                        s << m.at<double>(i,j);

                        if( matrixlayout )
                        {
                            if( j+1 == m.cols )
                            {
                                s << std::endl;
                            }
                            else
                            {
                                s << ' ';
                            }
                        }
                        else
                        {
                            s << std::endl;
                        }
                    }
                }
            };

            s << "Reprojection error: " << err << std::endl;
            s << std::endl;

            s << "Camera matrix:" << std::endl;
            write_mat( calibration.calibration_matrix, true );
            s << std::endl;

            s << "Distortion coefficients:" << std::endl;
            write_mat( calibration.distortion_coefficients, false );
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

