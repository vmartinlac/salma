#include <QJsonDocument>
#include "SLAMProject.h"
#include <QFile>
#include <QByteArray>
#include "VideoSystem.h"

SLAMProject::SLAMProject()
{
}

SLAMProject::~SLAMProject()
{
}

bool SLAMProject::load(const char* path)
{
    QFile file;
    QJsonDocument doc;

    bool ok = true;
    const char* error = "";

    mLeftCamera.reset(new CameraCalibrationData());
    mRightCamera.reset(new CameraCalibrationData());
    mStereoRig.reset(new StereoRigCalibrationData());

    if(ok)
    {
        ok = mDir.cd(path);
        error = "Path does not exist!";
    }

    if(ok)
    {
        ok = mDir.exists("video/recording.csv");
        error = "video/recording.csv does not exist!";
    }

    if(ok)
    {
        const QString path = mDir.absoluteFilePath("config.json");
        file.setFileName(path);
        ok = file.open(QFile::ReadOnly);
        error = "Could not open config.json";
    }

    if(ok)
    {
        QByteArray buffer = file.readAll();
        file.close();

        doc = QJsonDocument::fromJson(buffer);
        ok = doc.isObject();
        error = "config.json is not a valid json document!";
    }

    if(ok)
    {
        mParameters = doc.object();
    }

    if(ok)
    {
        const std::string path = mDir.absoluteFilePath("left_camera.json").toStdString();
        ok = mLeftCamera->loadFromFile(path);
        error = "Could not load left camera calibration data!";
    }

    if(ok)
    {
        const std::string path = mDir.absoluteFilePath("right_camera.json").toStdString();
        ok = mRightCamera->loadFromFile(path);
        error = "Could not load right camera calibration data!";
    }

    if(ok)
    {
        ok = ( mLeftCamera->image_size == mRightCamera->image_size );
        error = "Sizes of left camera and right camera differ!";
    }

    if(ok)
    {
        const std::string path = mDir.absoluteFilePath("rig.json").toStdString();
        ok = mStereoRig->loadFromFile(path);
        error = "Could not load stereo rig calibration data!";
    }

    if(ok)
    {
        const std::string path = mDir.absoluteFilePath("video/").toStdString();
        mVideo = VideoSystem::instance()->createVideoSourceFromFileStereo(path);
        ok = bool(mVideo);
        error = "Could not initialize video!";
    }

    if( ok == false )
    {
        std::cerr << error << std::endl;
    }

    return ok;
}

VideoSourcePtr SLAMProject::getVideo()
{
    return mVideo;
}

CameraCalibrationDataPtr SLAMProject::getLeftCameraCalibration()
{
    return mLeftCamera;
}

CameraCalibrationDataPtr SLAMProject::getRightCameraCalibration()
{
    return mRightCamera;
}

StereoRigCalibrationDataPtr SLAMProject::getStereoRigCalibration()
{
    return mStereoRig;
}

bool SLAMProject::getParameterBoolean(const char* name, bool default_value)
{
    return mParameters[name].toBool(default_value);
}

int SLAMProject::getParameterInteger(const char* name, int default_value)
{
    return mParameters[name].toInt(default_value);
}

double SLAMProject::getParameterReal(const char* name, double default_value)
{
    return mParameters[name].toDouble(default_value);
}

