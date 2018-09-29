#include "RecordingThread.h"

void RecordingThread::run()
{
    mNumFrames = 0;

    if( mCamera )
    {
        while(isInterruptionRequested() == false)
        {
            Image image;
            mCamera->read(image);

            if(image.isValid())
            {
                ;
            }
        }
    }
}

