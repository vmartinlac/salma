#include <QFile>
#include <QStandardPaths>
#include "SLAMParameters.h"

bool SLAMParameters::loadFromJson(const QJsonDocument& doc)
{
    return false;
}

bool SLAMParameters::saveToJson(const QJsonDocument& doc)
{
    return false;
}

bool SLAMParameters::loadFromFile(const QString& path)
{
    QFile file(path);

    bool ok = file.isOpen();

    if(ok)
    {
        QByteArray content = file.readAll();
        QJsonDocument doc = QJsonDocument::fromJson(content);
        ok = loadFromJson(doc);
    }

    return ok;
}

bool SLAMParameters::saveToFile(const QString& path)
{
    QJsonDocument doc;
    bool ok = saveToJson(doc);

    if(ok)
    {
        QByteArray buffer = doc.toJson();

        QFile file(path);

        ok = file.isOpen();

        if(ok)
        {
            ok = ( file.write(buffer) == buffer.size() );
            file.close();
        }
    }

    return ok;
}

QString SLAMParameters::getDefaultParameterFileName()
{
    return QStandardPaths::locate(
        QStandardPaths::AppConfigLocation,
        "slam_parameters.json");
}

bool SLAMParameters::loadFromDefaultParameterFile()
{
    return loadFromFile( getDefaultParameterFileName() );
}

bool SLAMParameters::saveToDefaultParameterFile()
{
    return saveToFile( getDefaultParameterFileName() );
}

void SLAMParameters::reset()
{
    calibration_matrix = ( cv::Mat_<float>(3,3) << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 );
    distortion_coefficients = ( cv::Mat_<float>(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0 );
    calibration_target_scale = 1.0;
    patch_size = 12;
    num_depth_hypotheses = 100;
    min_distance_to_camera = 0.0;
}

SLAMParameters::SLAMParameters()
{
    reset();
}

