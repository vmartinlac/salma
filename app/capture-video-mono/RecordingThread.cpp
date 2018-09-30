#include "RecordingThread.h"
#include <opencv2/imgcodecs.hpp>
#include <iostream>

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
        params.camera->open();

        while(isInterruptionRequested() == false)
        {
            Image image;
            params.camera->read(image);

            if(image.isValid())
            {
                QString filename = params.output_directory.absoluteFilePath(QString("%1.bmp").arg(QString::number(mNumFrames), 6, '0'));
                //cv::imwrite(filename.toLocal8Bit().data(), image.refFrame());
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
    }
}

