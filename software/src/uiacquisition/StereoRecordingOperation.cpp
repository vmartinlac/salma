#include <QThread>
#include <thread>
#include <opencv2/imgcodecs.hpp>
#include <sstream>
#include <iostream>
#include <fstream>
#include "StereoRecordingOperation.h"
#include "Image.h"

StereoRecordingOperation::StereoRecordingOperation()
{
    mVisualizationOnly = false;
    mMaxFrameRate = 1000;
}

StereoRecordingOperation::~StereoRecordingOperation()
{
}

bool StereoRecordingOperation::before()
{
    bool ok = true;

    mClock.start();
    mNumFrames = 0;

    if(ok)
    {
        ok = bool(mCamera) && (mCamera->getNumberOfCameras() == 2);
    }

    if( mVisualizationOnly == false )
    {
        if(ok)
        {
            const QString csv_path = mOutputDirectory.absoluteFilePath("recording.csv");
            mOutputCSV.open( csv_path.toLocal8Bit().data(), std::ofstream::out );
            ok = mOutputCSV.is_open();
        }
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

bool StereoRecordingOperation::step()
{
    bool ret = true;

    if( mCamera)
    {
        Image image;

        mCamera->read(image);
        mCamera->trigger();

        if( image.isValid() && image.getNumberOfFrames() == 2 )
        {
            if( mVisualizationOnly == false )
            {
                const QString left_basename = QString("frame_%1_left.bmp").arg(QString::number(mNumFrames), 6, '0');
                const QString right_basename = QString("frame_%1_right.bmp").arg(QString::number(mNumFrames), 6, '0');

                const QString left_filename = mOutputDirectory.absoluteFilePath(left_basename);
                const QString right_filename = mOutputDirectory.absoluteFilePath(right_basename);

                cv::imwrite(left_filename.toLocal8Bit().data(), image.getFrame(0));
                cv::imwrite(right_filename.toLocal8Bit().data(), image.getFrame(1));

                mOutputCSV << mNumFrames << " ";
                mOutputCSV << image.getTimestamp() << " ";
                mOutputCSV << left_basename.toLocal8Bit().data() << " ";
                mOutputCSV << right_basename.toLocal8Bit().data() << std::endl;
            }

            mNumFrames++;

            // display video.
            {
                Image concat;
                image.concatenate(concat);

                if( concat.isValid() == false ) throw std::runtime_error("some unexpected error");

                mVideoPort->beginWrite();
                mVideoPort->data().image = concat.getFrame();
                mVideoPort->endWrite();
            }

            // write output text.
            {
                const int total_seconds = static_cast<int>( mClock.elapsed()*1.0e-3 );
                const int seconds = total_seconds % 60;
                const int minutes = total_seconds / 60;

                std::stringstream s;

                s << "Frame count: " << mNumFrames << std::endl;
                s << "Left image size: " << image.getFrame(0).cols << " * " << image.getFrame(0).rows << std::endl;
                s << "Right image size: " << image.getFrame(1).cols << " * " << image.getFrame(1).rows << std::endl;
                s << "Recording duration: " << minutes << " min " << seconds << " seconds" << std::endl;
                s << std::endl;
                s << "Camera name: " << mCamera->getHumanName() << std::endl;
                s << "Output directory: " << mOutputDirectory.path().toStdString() << std::endl;
                s << "Visualization only: " << (mVisualizationOnly ? "true" : "false") << std::endl;
                s << "Max frame rate: " << mMaxFrameRate << std::endl;

                mStatsPort->beginWrite();
                mStatsPort->data().text = s.str().c_str();
                mStatsPort->endWrite();
            }

            if(mMaxFrameRate > 0 && mNumFrames > 1)
            {
                const int diff = 1000/mMaxFrameRate - mFrameRateClock.elapsed();
                if( diff > 0 )
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(diff));
                }
            }

            mFrameRateClock.start();
        }
    }
    else
    {
        ret = false;
    }

    return ret;
}

void StereoRecordingOperation::after()
{
    if( mVisualizationOnly == false )
    {
        mOutputCSV.close();
    }

    if(mCamera)
    {
        mCamera->close();
    }
}

