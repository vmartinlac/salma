#include <opencv2/imgcodecs.hpp>
#include <QDateTime>
#include <iomanip>
#include <sstream>
#include "SLAMDebug.h"

SLAMDebug::SLAMDebug( SLAMConfigurationPtr config )
{
    mImageCount = 0;
    mConfiguration = std::move(config);

    mOpenCVTypes[CV_32FC1] = "CV_32FC1";
    mOpenCVTypes[CV_32FC2] = "CV_32FC2";
    mOpenCVTypes[CV_32FC3] = "CV_32FC3";

    mOpenCVTypes[CV_64FC1] = "CV_64FC1";
    mOpenCVTypes[CV_64FC2] = "CV_64FC2";
    mOpenCVTypes[CV_64FC3] = "CV_64FC3";

    mOpenCVTypes[CV_8UC1] = "CV_8UC1";
    mOpenCVTypes[CV_8UC2] = "CV_8UC2";
    mOpenCVTypes[CV_8UC3] = "CV_8UC3";

    mOpenCVTypes[CV_8SC1] = "CV_8SC1";
    mOpenCVTypes[CV_8SC2] = "CV_8SC2";
    mOpenCVTypes[CV_8SC3] = "CV_8SC3";

    mOpenCVTypes[CV_16UC1] = "CV_16UC1";
    mOpenCVTypes[CV_16UC2] = "CV_16UC2";
    mOpenCVTypes[CV_16UC3] = "CV_16UC3";

    mOpenCVTypes[CV_16SC1] = "CV_16SC1";
    mOpenCVTypes[CV_16SC2] = "CV_16SC2";
    mOpenCVTypes[CV_16SC3] = "CV_16SC3";

    mOpenCVTypes[CV_32SC1] = "CV_32SC1";
    mOpenCVTypes[CV_32SC2] = "CV_32SC2";
    mOpenCVTypes[CV_32SC3] = "CV_32SC3";
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

std::string SLAMDebug::describeOpenCVMat(const cv::Mat& mat)
{
    std::string type;

    std::map<int,std::string>::iterator it = mOpenCVTypes.find(mat.type());
    if(it == mOpenCVTypes.end())
    {
        type = "unknown";
    }
    else
    {
        type = it->second;
    }

    std::stringstream s;
    s << "Mat (cols, rows) = (" << mat.cols << ", " << mat.rows << ") type = " << type;

    return s.str();
}

