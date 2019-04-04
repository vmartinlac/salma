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

    if( mVisualizationOnly == false )
    {
        mResult.reset(new RecordingHeader());
        mResult->id = -1;
        mResult->name = mRecordingName;
        mResult->date.clear();
        mResult->timestamps.clear();
        mResult->views.clear();
        mSuccess = project->createRecordingDirectory(mResult->directory);
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
            else
            {
                bool first_frame = false;

                if(mSuccess)
                {
                    mSuccess = ( image.getNumberOfFrames() == mCamera->getNumberOfCameras() );
                }

                if(mSuccess)
                {
                    first_frame = mResult->timestamps.empty();
                    mResult->timestamps.push_back(image.getTimestamp());
                }

                for(int v=0; mSuccess && v<image.getNumberOfFrames(); v++)
                {
                    cv::Mat frame = image.getFrame(v);

                    if( first_frame )
                    {
                        mResult->views.push_back(frame.size());
                    }
                    else
                    {
                        mSuccess = ( frame.size() == mResult->views[v] );
                    }

                    if(mSuccess)
                    {
                        const std::string fname = mResult->getImageFileName(mNumFrames, v).toStdString();
                        mSuccess = syncimwrite(fname, frame);
                    }
                }

                if(mSuccess)
                {
                    mNumFrames++;
                    if( mNumFrames != mResult->num_frames() ) throw std::runtime_error("internal error");
                }
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
}

void RecordingOperation::uiafter(QWidget* parent, Project* project)
{
    const char* err = "Error while recording!";

    if( mVisualizationOnly == false )
    {
        int recording_id = -1;

        if( bool(mResult) == false ) throw std::runtime_error("internal error");

        if(mSuccess)
        {
            mSuccess = project->saveRecording(mResult, recording_id);
            err = "Recording could not be saved to database!";
        }

        if(mSuccess == false)
        {
            mResult->directory.removeRecursively();
        }
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

const char* RecordingOperation::getName()
{
    return "Recording";
}

