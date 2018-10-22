#include <QJsonDocument>
#include <QFile>
#include <QJsonObject>
#include <QJsonArray>
#include <fstream>
#include <iostream>
#include "Serialization.h"
#include "CameraCalibrationData.h"

CameraCalibrationData::CameraCalibrationData()
{
}

bool CameraCalibrationData::saveToFile(const std::string& path)
{
    bool ok = true;

    QJsonDocument doc;

    try
    {
        QJsonObject obj;
        obj["calibration_matrix"] = Serialization::serializeCalibrationMatrix(calibration_matrix);
        obj["distortion_coefficients"] = Serialization::serializeDistortionCoefficients(distortion_coefficients);
        obj["image_size"] = Serialization::serializeSize(image_size);

        doc.setObject(obj);
    }
    catch(SerializationError& err)
    {
        ok = false;
    }

    QFile file;

    if(ok)
    {
        file.setFileName(path.c_str());
        ok = file.open(QIODevice::WriteOnly);
    }

    if(ok)
    {
        file.write(doc.toJson());
        file.close();
    }

    return ok;
}

bool CameraCalibrationData::loadFromFile(const std::string& path)
{
    bool ok = true;
    QFile file;
    QByteArray buff;
    QJsonDocument doc;
    QJsonObject root;

    if(ok)
    {
        file.setFileName(path.c_str());
        ok = file.open(QIODevice::ReadOnly);
    }

    if(ok)
    {
        buff = file.readAll();
        file.close();
    }

    if(ok)
    {
        doc = QJsonDocument::fromJson(buff);
        ok = (doc.isNull() == false) && doc.isObject();
    }

    if(ok)
    {
        root = doc.object();
        try
        {
            calibration_matrix = Serialization::deserializeCalibrationMatrix(root["calibration_matrix"]);
            distortion_coefficients = Serialization::deserializeDistortionCoefficients(root["distortion_coefficients"]);
            image_size = Serialization::deserializeSize(root["image_size"]);
        }
        catch(SerializationError& err)
        {
            ok = false;
        }
    }

    return ok;
}

