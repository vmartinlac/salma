#include <opencv2/imgcodecs.hpp>
#include <QDateTime>
#include <iomanip>
#include <sstream>
#include "SLAMDebug.h"

SLAMDebug::SLAMDebug( SLAMConfigurationPtr config )
{
    mImageCount = 0;
    mConfiguration = std::move(config);
}

SLAMDebug::~SLAMDebug()
{
}

bool SLAMDebug::init()
{
    bool ok = true;

    const QString dirname = QString("salma_debug_") + QDateTime::currentDateTime().toString("dd.MM.yyyy_hh.mm.ss");

    mImageCount = 0;

    if(ok)
    {
        mDir = QDir();
        ok = mDir.mkdir(dirname);
    }

    if(ok)
    {
        ok = mDir.cd(dirname);
    }

    return ok;
}

void SLAMDebug::saveImage(int frame, const std::string& name, const cv::Mat& image)
{
    const QString fname = QString("%1_frame%2_%3.png").arg(mImageCount, 6, 10, QChar('0')).arg(frame).arg(name.c_str());
    const QString fpath = mDir.absoluteFilePath(fname);

    cv::imwrite(fpath.toLocal8Bit().data(), image);

    mImageCount++;
}

