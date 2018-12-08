#include <fstream>
#include <QJsonDocument>
#include <QFile>
#include <QByteArray>
#include "SLAMProject.h"
#include "SLAMReconstructionDB.h"
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

bool SLAMProject::exportReconstruction(FramePtr last_frame, const std::string& name)
{
    /*
    QString effective_name;
    QDir my_dir = mDir;
    bool ok = true;

    if(ok)
    {
        ok = (name.empty() == false);
    }

    if(ok)
    {
        my_dir.mkdir("reconstructions"); // if this directory already exists, nothing is done.
        ok = my_dir.cd("reconstructions");
    }

    if(ok)
    {
        QString base_name = name.c_str();
        int k = 2;

        effective_name = base_name;

        while(my_dir.exists(effective_name))
        {
            effective_name = base_name + QString::number(k);
            k++;
        }

        ok = my_dir.mkdir(effective_name);
    }

    if(ok)
    {
        ok = my_dir.cd(effective_name);
    }

    if(ok)
    {
        QString path = my_dir.absoluteFilePath("hello.txt");

        std::ofstream f(path.toLocal8Bit().constData());
        f << "hello" << std::endl;
        f.close();
    }

    if(ok == false)
    {
        std::cout << "Error during export of reconstruction." << std::endl;
    }
    return ok;

    */

    SLAMReconstructionDB db;
    bool ok = true;

    if(ok)
    {
        ok = db.open( mDir.absoluteFilePath("reconstructions.sqlite").toStdString() );
    }

    if(ok)
    {
        ok = db.saveReconstruction(last_frame, name);
    }

    if(ok)
    {
        db.close();
    }

    if(ok == false)
    {
        std::cout << "Error when saving reconstruction!" << std::endl;
    }

    return ok;
}

