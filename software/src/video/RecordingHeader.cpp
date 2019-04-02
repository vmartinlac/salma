#include "RecordingHeader.h"

QString RecordingHeader::getImageFileName(int frame, int view)
{
    if( 0 <= frame && frame < num_frames() && 0 <= view && view < num_views() )
    {
        const QString basename = "frame_" + QString::number(frame) + "_" + QString::number(view) + ".jpg";

        return directory.absoluteFilePath(basename);
    }
    else
    {
        throw std::runtime_error("internal error");
    }
}
