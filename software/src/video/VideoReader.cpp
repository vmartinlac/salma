#include <opencv2/imgcodecs.hpp>
#include <QDir>
#include <QString>
#include <QStringList>
#include "VideoReader.h"

VideoReader::VideoReader(int num_views=1)
{
    mNumViews = num_views;
    mTriggered = false;
}

VideoReader::~VideoReader()
{
}

void VideoReader::setPath(const std::string& path)
{
    mDirectory = QDir(path.c_str());
}

std::string VideoReader::getHumanName()
{
    std::string path = mDirectory.absolutePath().toStdString();

    std::string ret;

    if(mNumViews == 1)
    {
        ret = "{ MONO STREAM " + path + " }";
    }
    else if(mNumViews == 2)
    {
        ret = "{ STEREO STREAM " + path + " }";
    }
    else
    {
        ret = "{ " + path + " }";
    }

    return ret;
}

bool VideoReader::open()
{
    std::string listing = mDirectory.absoluteFilePath( "recording.csv" ).toStdString();
    mCSVFile.open(listing);
    return mCSVFile.is_open();
}

void VideoReader::close()
{
    mCSVFile.close();
}

void VideoReader::trigger()
{
    auto proc = [this] ()
    {
        Image image;
        std::string line;
        double timestamp = 0.0;
        std::vector<cv::Mat> frames(mNumViews);
        bool ok = true;

        std::getline(mCSVFile, line);

        QStringList tokens = QString(line.c_str()).split(" ");
        ok = tokens.size() == (2 + mNumViews);

        if(ok)
        {
            timestamp = tokens[1].toDouble(&ok);
        }

        if(ok)
        {
            for(int i=0; ok && i<mNumViews; i++)
            {
                std::string filename = mDirectory.absoluteFilePath( tokens[2+i] ).toStdString();
                frames[i] = cv::imread(filename);
                ok = ok && (frames[i].data != nullptr);
            }
        }

        if(ok)
        {
            image.setValid(timestamp, frames);
        }
        else
        {
            image.setInvalid();
        }

        return image;
    };

    mNextImage = std::async(std::launch::async, proc);
    mTriggered = true;
}

void VideoReader::read(Image& image)
{
    if(mTriggered == false)
    {
        trigger();
    }

    image = mNextImage.get();

    mTriggered = false;
}

int VideoReader::getNumberOfCameras()
{
    return mNumViews;
}

