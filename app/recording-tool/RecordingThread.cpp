#include "RecordingThread.h"
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <fstream>

RecordingThread::RecordingThread(Parameters* parameters, Output* output, QObject* parent) : QThread(parent)
{
    mParameters = parameters;
    mOutput = output;
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
    ParametersData params;
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

                mOutput->beginWrite();
                mOutput->data().camera_name = params.camera->getHumanName().c_str();
                mOutput->data().output_directory = params.output_directory.path();
                mOutput->data().image = image.refFrame();
                mOutput->data().frame_count = mNumFrames;
                mOutput->endWrite();
            }
        }

        params.camera->close();

        csv.close();
    }
}

