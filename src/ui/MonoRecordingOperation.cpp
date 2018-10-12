#include "MonoRecordingOperation.h"

MonoRecordingOperation::MonoRecordingOperation()
{
}

MonoRecordingOperation::~MonoRecordingOperation()
{
}

void MonoRecordingOperation::before()
{
}

bool MonoRecordingOperation::step()
{
    return false;
}

void MonoRecordingOperation::after()
{
}

/*
#include "RecordingThread.h"
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <fstream>

RecordingThread::RecordingThread(
    RecordingParameters* parameters,
    VideoInputPort* video,
    RecordingStatsInputPort* stats,
    QObject* parent) : QThread(parent)
{
    mParameters = parameters;
    mVideo = video;
    mStats = stats;
    mNumFrames = 0;
}

RecordingThread::~RecordingThread()
{
    if(isRunning())
    {
        requestInterruption();
        wait();
    }
}

void RecordingThread::run()
{
    RecordingParametersData params;
    mParameters->read(params);

    mNumFrames = 0;

    if( params.camera )
    {
        QString csv_path = params.output_directory.absoluteFilePath("recording.csv");
        std::ofstream csv(csv_path.toLocal8Bit().data(), std::ofstream::out);

        params.camera->open();

        while(isInterruptionRequested() == false)
        {
            Image image;
            params.camera->read(image);

            if(image.isValid())
            {
                QString basename = QString("%1.bmp").arg(QString::number(mNumFrames), 6, '0');
                QString filename = params.output_directory.absoluteFilePath(basename);
                cv::imwrite(filename.toLocal8Bit().data(), image.refFrame());

                csv << mNumFrames << " " << basename.toLocal8Bit().data() << " " << image.getTimestamp() << std::endl;

                mNumFrames++;

                mStats->beginWrite();
                mStats->data().camera_name = params.camera->getHumanName();
                mStats->data().output_directory = params.output_directory.path().toStdString();
                mStats->data().frame_count = mNumFrames;
                mStats->data().image_width = image.refFrame().rows;
                mStats->data().image_height = image.refFrame().cols;
                mStats->endWrite();

                mVideo->beginWrite();
                mVideo->data().image = image.refFrame();
                mVideo->endWrite();
            }
        }

        params.camera->close();

        csv.close();
    }
}
*/

