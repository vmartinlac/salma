#include <QMessageBox>
#include "ReconstructionOperation.h"

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

    if(ok)
    {
        ok = ( mReconstructionName.empty() == false && bool(mRecordingHeader) && bool(mCalibration) && bool(mConfiguration) );
    }

    if(ok)
    {
        mRecordingReader.reset(new RecordingReader(mRecordingHeader, true));
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
    Image image;

    mRecordingReader->read(image);
    mRecordingReader->trigger();

    if(image.isValid())
    {
        mEngine->processFrame(image);
    }

    return true;
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
        // TODO: save the reconstruction into database.
    }
}

