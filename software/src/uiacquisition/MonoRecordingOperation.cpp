#include <QThread>
#include <opencv2/imgcodecs.hpp>
#include <thread>
#include <sstream>
#include <iostream>
#include <fstream>
#include "MonoRecordingOperation.h"
#include "Image.h"

MonoRecordingOperation::MonoRecordingOperation()
{
    mVisualizationOnly = false;
    mMaxFrameRate = 1000;
}

MonoRecordingOperation::~MonoRecordingOperation()
{
}

bool MonoRecordingOperation::before()
{
    bool ok = true;

    mClock.start();
    mNumFrames = 0;

    if(ok)
    {
        ok = bool(mCamera);
    }

    if(mVisualizationOnly == false)
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

bool MonoRecordingOperation::step()
{
    bool ret = true;

    if( mCamera)
    {
        Image image;
        mCamera->read(image);
        mCamera->trigger();

        if(image.isValid())
        {
            if(mVisualizationOnly == false)
            {
                const QString basename = QString("%1.bmp").arg(QString::number(mNumFrames), 6, '0');
                const QString filename = mOutputDirectory.absoluteFilePath(basename);

                cv::imwrite(filename.toLocal8Bit().data(), image.getFrame());
                mOutputCSV << mNumFrames << " " << basename.toLocal8Bit().data() << " " << image.getTimestamp() << std::endl;
            }

            mNumFrames++;

            // write to ports.
            {
                const int total_seconds = static_cast<int>( mClock.elapsed()*1.0e-3 );
                const int seconds = total_seconds % 60;
                const int minutes = total_seconds / 60;

                std::stringstream s;

                s << "Frame count: " << mNumFrames << std::endl;
                s << "Image resolution: " << image.getFrame().cols << " x " << image.getFrame().rows << std::endl;
                s << "Recording duration: " << minutes << " min " << seconds << " seconds" << std::endl;
                s << std::endl;
                s << "Camera name: " << mCamera->getHumanName() << std::endl;
                s << "Output directory: " << mOutputDirectory.path().toStdString() << std::endl;
                s << "Visualization only: " << (mVisualizationOnly ? "true" : "false" ) << std::endl;
                s << "Max frame rate: " << mMaxFrameRate << std::endl;

                mStatsPort->beginWrite();
                mStatsPort->data().text = s.str().c_str();
                mStatsPort->endWrite();

                mVideoPort->beginWrite();
                mVideoPort->data().image = image.getFrame();
                mVideoPort->endWrite();
            }

            if(mMaxFrameRate > 0 && mNumFrames > 1)
            {
                const int diff = 1000/mMaxFrameRate - mFrameRateClock.elapsed();
                if( diff > 0 );
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

    QThread::msleep(1000/20);

    return ret;
}

void MonoRecordingOperation::after()
{
    if(mVisualizationOnly == false)
    {
        mOutputCSV.close();
    }

    if(mCamera)
    {
        mCamera->close();
    }
}

