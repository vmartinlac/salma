#include <iostream>
#include <future>
#include <thread>
#include <sstream>
#include <fstream>
#include <QThread>
#include <QTime>
#include <sophus/average.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include "Image.h"
#include "Tracker.h"
#include "StereoRigCalibrationOperation.h"
#include "StereoRigCalibrationData.h"

StereoRigCalibrationOperation::StereoRigCalibrationOperation()
{
    mNumberOfCalibrationPoses = 20;
    mMillisecondsOfTemporisation = 700;
}

StereoRigCalibrationOperation::~StereoRigCalibrationOperation()
{
}

bool StereoRigCalibrationOperation::before()
{
    mFrameCount = 0;
    mClock.start();
    mPoses.clear();

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

bool StereoRigCalibrationOperation::step()
{
    bool ret = true;

    bool can_calibrate = false;

    Image image;
    image.setInvalid();

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
        if( mPoses.empty() || mClock.elapsed() > mMillisecondsOfTemporisation )
        {
			std::future<bool> left_target_found = std::async( std::launch::async, [this, &image] () -> bool
			{
				return mLeftTracker.track(image.getFrame(0), true);
			});

			std::future<bool> right_target_found = std::async( std::launch::async, [this, &image] () -> bool
			{
				return mRightTracker.track(image.getFrame(1), true);
			});

			if( left_target_found.get() && right_target_found.get() )
            {

				/*
				TODO: compute geometric configuration.
				*/

                mClock.start();

                can_calibrate = ( mPoses.size() >= mNumberOfCalibrationPoses );
            }
        }

        mFrameCount++;

        mVideoPort->beginWrite();
        mVideoPort->data().image = image.getFrame(0); // TODO: output both frames.
        mVideoPort->endWrite();

        writeOutputText();
    }

    if( can_calibrate )
    {
		Sophus::optional< Sophus::SE3<double> > right_camera_to_left_camera;
        StereoRigCalibrationData calibration;
		const char* error_message = "";
		bool ok = true;

		if(ok)
		{
			right_camera_to_left_camera = Sophus::average( mPoses );
			ok = bool(right_camera_to_left_camera);
			error_message = "Could average calibration data.";
		}

		if(ok)
		{
			// TODO: compute calibration.left_camera_to_world and calibration.right_camera_to_world.

			// TODO: compute standard deviation on attitude and position.
			throw;
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
            s << std::endl;

            s << "Right camera to world transformation:" << std::endl;
            s << std::endl;

            s << "Standard deviation on attitude: " << std::endl;
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

        ret = false;
    }

    QThread::msleep(5);

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
    /*
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
    */
}

