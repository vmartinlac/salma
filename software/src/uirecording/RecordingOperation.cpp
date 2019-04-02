#include <QMessageBox>
#include <QFile>
#include <QThread>
#include <opencv2/imgcodecs.hpp>
#include <thread>
#include <sstream>
#include <iostream>
#include <fstream>
#include "RecordingOperation.h"
#include "Image.h"
#include "SyncIO.h"
#include "Project.h"

/*
RecordingOperation::RecordingOperation()
{
    mVisualizationOnly = false;
    mMaxFrameRate = 1000;
    mSoftwareTrigger = true;
}

RecordingOperation::~RecordingOperation()
{
}

bool RecordingOperation::uibefore(QWidget* parent, Project* project)
{
    mSuccess = true;
    mVideoFileCreated = false;
    mVideoFilename.clear();

    if( mVisualizationOnly == false )
    {
        mSuccess = project->getNewRecordingFilename(mVideoFilename) && (mVideoFilename.empty() == false);
    }

    return mSuccess;
}

bool RecordingOperation::before()
{
    mClock.start();
    mNumFrames = 0;

    if(mSuccess)
    {
        mSuccess = (mRecordingName.empty() == false && mMaxFrameRate > 0.0);
    }

    if(mSuccess)
    {
        mSuccess = bool(mCamera) && (mCamera->getNumberOfCameras() >= 1);
    }

    if(mSuccess)
    {
        if(mVisualizationOnly)
        {
            mResult.reset();
        }
        else
        {
            mResult.reset(new RecordingHeader());
            mResult->id = -1;
            mResult->name = mRecordingName;
            mResult->date.clear();
            mResult->timestamps.clear();
            mResult->views.clear();
            mResult->filename = mVideoFilename;
        }
    }

    if(mSuccess)
    {
        mSuccess = mCamera->open();
    }

    if(mSuccess && mSoftwareTrigger)
    {
        mCamera->trigger();
    }

    return mSuccess;
}

bool RecordingOperation::step()
{
    if( mSuccess )
    {
        Image image;

        mCamera->read(image);

        if(mSoftwareTrigger)
        {
            mCamera->trigger();
        }

        if( image.isValid() )
        {
            if( mVisualizationOnly )
            {
                mNumFrames++;
            }
            else if(mVideoWriter.isOpened())
            {
                if(mSuccess)
                {
                    mSuccess = ( image.getNumberOfFrames() == mResult->num_views() );
                }

                if(mSuccess)
                {
                    if(mBuffer.size() != mResult->size) throw std::runtime_error("internal error");

                    for(int v=0; mSuccess && v<image.getNumberOfFrames(); v++)
                    {
                        const cv::Mat frame = image.getFrame(v);

                        if( mBuffer.type() != frame.type() ) throw std::runtime_error("internal error");

                        if( frame.size() == mResult->views[v].size() )
                        {
                            //std::cout << frame.type() << " " << mBuffer.type() << std::endl;
                            frame.copyTo( mBuffer( mResult->views[v] ) );
                        }
                        else
                        {
                            mSuccess = false;
                        }
                    }
                }

                if(mSuccess)
                {
                    if(mBuffer.type() != CV_8UC3 || mBuffer.size() != mResult->size) throw std::runtime_error("internal error");

                    mVideoWriter.write(mBuffer);
                    mResult->timestamps.push_back(image.getTimestamp());
                    mNumFrames++;

                    mSuccess = (mResult->timestamps.size() == mNumFrames);
                }
            }
            else
            {
                mResult->size.width = 0;
                mResult->size.height = 0;

                if( mResult->views.empty() == false ) throw std::runtime_error("internal error");

                for(int i=0; i<image.getNumberOfFrames(); i++)
                {
                    const cv::Mat frame = image.getFrame(i);

                    mResult->views.emplace_back(
                        mResult->size.width,
                        0,
                        frame.cols,
                        frame.rows);

                    mResult->size.width += frame.cols;
                    mResult->size.height = std::max(mResult->size.height, frame.rows);
                }

                mBuffer.create(mResult->size, CV_8UC3);

                const int fourcc = cv::VideoWriter::fourcc('M','J','P','G');
                //const int fourcc = cv::VideoWriter::fourcc('X','2','6','4');

                mSuccess = mVideoWriter.open(
                    mVideoFilename,
                    fourcc,
                    15.0,
                    mResult->size,
                    true);

                mVideoFileCreated = mSuccess;
            }

            // display video.
            if(image.getNumberOfFrames() > 1)
            {
                Image concat;
                image.concatenate(concat);

                if( concat.isValid() == false ) throw std::runtime_error("some unexpected error");

                videoPort()->beginWrite();
                videoPort()->data().image = concat.getFrame();
                videoPort()->endWrite();
            }
            else
            {
                videoPort()->beginWrite();
                videoPort()->data().image = image.getFrame();
                videoPort()->endWrite();
            }

            // write output text.
            if(mSuccess)
            {
                const int total_seconds = static_cast<int>( mClock.elapsed()*1.0e-3 );
                const int seconds = total_seconds % 60;
                const int minutes = total_seconds / 60;

                std::stringstream s;

                s << "Frame count: " << mNumFrames << std::endl;
                s << "Recording duration: " << minutes << " min " << seconds << " seconds" << std::endl;
                s << std::endl;

                s << "Camera name: " << mCamera->getHumanName() << std::endl;
                for(int v=0; v<image.getNumberOfFrames(); v++)
                {
                    s << "Image #" << v << "  size: " << image.getFrame(v).cols << " * " << image.getFrame(v).rows << std::endl;
                }
                s << std::endl;

                if(mVisualizationOnly == false)
                {
                    s << "Output filename: " << mVideoFilename << std::endl;
                }
                s << "Visualization only: " << (mVisualizationOnly ? "true" : "false") << std::endl;
                s << "Max frame rate: " << mMaxFrameRate << std::endl;

                statsPort()->beginWrite();
                statsPort()->data().text = s.str().c_str();
                statsPort()->endWrite();
            }
            else
            {
                statsPort()->beginWrite();
                statsPort()->data().text = "Error while saving the image!";
                statsPort()->endWrite();
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

    return mSuccess;
}

void RecordingOperation::after()
{
    if(mCamera)
    {
        mCamera->close();
    }

    if(mVideoWriter.isOpened())
    {
        mVideoWriter.release();
    }
}

void RecordingOperation::uiafter(QWidget* parent, Project* project)
{
    if( mVisualizationOnly == false )
    {
        int recording_id = -1;
        const char* err = "Error while recording!";

        if(mSuccess)
        {
            mSuccess = project->saveRecording(mResult, recording_id);
            err = "Recording could not be saved to database!";
        }

        if(mSuccess == false && mVideoFileCreated)
        {
            QFile(mVideoFilename.c_str()).remove();
        }

        if(mSuccess)
        {
            QMessageBox::information(parent, "Success", "Done recording!");
        }
        else
        {
            QMessageBox::critical(parent, "Error", err);
        }
    }
}

const char* RecordingOperation::getName()
{
    return "Recording";
}
*/

