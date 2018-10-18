#include <QThread>
#include <opencv2/imgcodecs.hpp>
#include <sstream>
#include <iostream>
#include <fstream>
#include "MonoRecordingOperation.h"
#include "Image.h"

MonoRecordingOperation::MonoRecordingOperation()
{
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

    if(ok)
    {
        const QString csv_path = mOutputDirectory.absoluteFilePath("recording.csv");
        mOutputCSV.open( csv_path.toLocal8Bit().data(), std::ofstream::out );
        ok = mOutputCSV.is_open();
    }

    if(ok)
    {
        ok = mCamera->open();
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

        if(image.isValid())
        {
            const QString basename = QString("%1.bmp").arg(QString::number(mNumFrames), 6, '0');
            const QString filename = mOutputDirectory.absoluteFilePath(basename);

            cv::imwrite(filename.toLocal8Bit().data(), image.getFrame());

            mOutputCSV << mNumFrames << " " << basename.toLocal8Bit().data() << " " << image.getTimestamp() << std::endl;

            mNumFrames++;

            // write to ports.
            {
                std::stringstream s;

                s << "Frame count: " << mNumFrames << std::endl;
                s << "Image resolution: " << image.getFrame().cols << " x " << image.getFrame().rows << std::endl;
                s << "Recording duration: " << double(mClock.elapsed())*1.0e-3 << std::endl;
                s << std::endl;
                s << "Camera name: " << mCamera->getHumanName() << std::endl;
                s << "Output directory: " << mOutputDirectory.path().toStdString() << std::endl;

                mStatsPort->beginWrite();
                mStatsPort->data().text = s.str().c_str();
                mStatsPort->endWrite();

                mVideoPort->beginWrite();
                mVideoPort->data().image = image.getFrame();
                mVideoPort->endWrite();
            }
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
    if(mCamera)
    {
        mCamera->close();
    }

    mOutputCSV.close();
}

