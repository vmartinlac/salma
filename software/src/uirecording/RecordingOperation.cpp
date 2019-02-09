#include <QMessageBox>
#include <QThread>
#include <thread>
#include <opencv2/imgcodecs.hpp>
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
}

RecordingOperation::~RecordingOperation()
{
}

bool RecordingOperation::uibefore(QWidget* parent, Project* project)
{
    bool ok = true;

    if(mVisualizationOnly)
    {
        mDirectory = QDir::temp();
    }
    else
    {
        ok = project->createRecordingDirectory(mDirectory);

        if(ok == false)
        {
            QMessageBox::critical(parent, "Error", "Could not create directory! Check filesystem permissions!");
        }
    }

    return ok;
}

bool RecordingOperation::before()
{
    bool ok = true;

    mClock.start();
    mNumFrames = 0;

    if(ok)
    {
        ok = (mRecordingName.empty() == false && mMaxFrameRate > 0.0);
    }

    if(ok)
    {
        ok = bool(mCamera) && (mCamera->getNumberOfCameras() >= 1);
    }

    if(ok)
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
            mResult->num_views = mCamera->getNumberOfCameras();
            mResult->num_frames = 0;
            mResult->directory = mDirectory;
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

bool RecordingOperation::step()
{
    bool ret = true;

    if( mCamera)
    {
        Image image;

        mCamera->read(image);
        mCamera->trigger();

        if( image.isValid() )
        {
            if( mVisualizationOnly )
            {
                mNumFrames++;
            }
            else
            {
                ret = ret && ( image.getNumberOfFrames() == mResult->num_views );

                if(ret)
                {
                    for(int v=0; ret && v<image.getNumberOfFrames(); v++)
                    {
                        const QString str_frame = QString::number(mNumFrames);
                        const QString str_view = QString::number(v);
                        const QString basename = QString("frame_%1_%2.png").arg(str_frame, 8, '0').arg(str_view, 2, '0');
                        const QString filename = mDirectory.absoluteFilePath(basename);
                        ret = ret && syncimwrite(filename.toLocal8Bit().data(), ".png", image.getFrame(v));

                        RecordingHeaderView rhv;
                        rhv.filename = basename;
                        mResult->views.push_back(rhv);
                    }

                    RecordingHeaderFrame rhf;
                    rhf.timestamp = image.getTimestamp();
                    mResult->frames.push_back(rhf);
                    mResult->num_frames++;

                    mNumFrames++;

                    ret = ret && (mResult->num_frames == mResult->frames.size());
                    ret = ret && (mResult->num_views*mResult->num_frames == mResult->views.size());
                    ret = ret && (mNumFrames == mResult->num_frames);
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
            if(ret)
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
                    s << "Output directory: " << mDirectory.dirName().toStdString() << std::endl;
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
    else
    {
        ret = false;
    }

    if(ret == false && bool(mResult))
    {
        mResult.reset();
        mDirectory.removeRecursively();
    }

    return ret;
}

void RecordingOperation::after()
{
    if(mCamera)
    {
        mCamera->close();
    }
}

const char* RecordingOperation::getName()
{
    return "Recording";
}

void RecordingOperation::uiafter(QWidget* parent, Project* project)
{
    if( mVisualizationOnly == false )
    {
        if( mResult )
        {
            bool ok = true;
            int recording_id = -1;

            if(ok)
            {
                ok = project->saveRecording(mResult, recording_id);
            }

            if(ok)
            {
                QMessageBox::information(parent, "Success", "Done recording!");
            }
            else
            {
                mDirectory.removeRecursively();
                QMessageBox::critical(parent, "Error", "Recording could not be saved to database!");
            }
        }
        else
        {
            QMessageBox::critical(parent, "Error", "Recording failed!");
        }
    }
}

