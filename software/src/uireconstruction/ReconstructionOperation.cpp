#include <QMessageBox>
#include "ReconstructionOperation.h"
#include "Project.h"

ReconstructionOperation::ReconstructionOperation()
{
}

ReconstructionOperation::~ReconstructionOperation()
{
}

const char* ReconstructionOperation::getName()
{
    return "Reconstruction";
}

bool ReconstructionOperation::uibefore(QWidget* parent, Project* project)
{
    return true;
}

bool ReconstructionOperation::before()
{
    bool ok = true;

    mReconstruction.reset();
    mNextFrame = 0;

    if(ok)
    {
        ok = ( mReconstructionName.empty() == false && bool(mRecordingHeader) && bool(mCalibration) && bool(mConfiguration) );
    }

    if(ok)
    {
        ok = ( mRecordingHeader->id >= 0 && mCalibration->id >= 0 );
    }

    if(ok)
    {
        mRecordingReader.reset(new RecordingReader(mRecordingHeader, false));
        mEngine.reset(new SLAMEngine());
        const bool ok = mEngine->initialize(mCalibration, mConfiguration);
    }

    if(ok)
    {
        ok = mRecordingReader->open();
    }

    if(ok)
    {
        mRecordingReader->trigger();
    }
    else
    {
        mEngine.reset();
        mRecordingReader.reset();
    }

    return ok;
}

bool ReconstructionOperation::step()
{
    bool ret = true;

    if(mNextFrame >= mRecordingHeader->num_frames)
    {
        ret = false;
    }
    else
    {
        Image image;

        mRecordingReader->seek(mNextFrame);
        mRecordingReader->read(image);
        mRecordingReader->trigger();

        if(image.isValid())
        {

            // set video output.
            {
                Image concat;
                image.concatenate(concat);

                videoPort()->beginWrite();
                if( concat.isValid() )
                {
                    videoPort()->data().image = concat.getFrame();
                }
                else
                {
                    videoPort()->data().image.create(640, 480, CV_8UC3);
                    videoPort()->data().image = cv::Scalar(64, 64, 64);
                }
                videoPort()->endWrite();
            }

            // set stats output.
            {
                std::stringstream s;
                s << "Processing frame " << mNextFrame << std::endl;

                statsPort()->beginWrite();
                statsPort()->data().text = s.str().c_str();
                statsPort()->endWrite();
            }

            mEngine->processFrame(mNextFrame, image);
        }

        mNextFrame++;
    }

    return ret;
}

void ReconstructionOperation::after()
{
    mEngine->finalize(mReconstruction);
    mEngine.reset();
    mRecordingReader->close();
}

void ReconstructionOperation::uiafter(QWidget* parent, Project* project)
{
    if( mReconstruction )
    {
        bool ok = true;
        int reconstruction_id = -1;

        if(ok)
        {
            mReconstruction->id = -1;
            mReconstruction->name = mReconstructionName;
            mReconstruction->recording = mRecordingHeader;
            mReconstruction->calibration = mCalibration;

            ok = project->saveReconstruction(mReconstruction, reconstruction_id);
        }

        if(ok)
        {
            QMessageBox::information(parent, "Success", "Reconstruction done!");
        }
        else
        {
            QMessageBox::critical(parent, "Error", "Failed to save reconstruction to database!");
        }
    }
    else
    {
        QMessageBox::critical(parent, "Error", "Reconstruction failed!");
    }
}

