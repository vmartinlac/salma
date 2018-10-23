#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <fstream>
#include <iostream>
#include "StereoRigCalibrationData.h"
#include "Serialization.h"

StereoRigCalibrationData::StereoRigCalibrationData()
{
}

bool StereoRigCalibrationData::saveToFile(const std::string& path)
{
    bool ok = true;

    QJsonDocument doc;

    try
    {
        QJsonObject obj;
        obj["left_camera_to_world"] = Serialization::serializePose(left_camera_to_world);
        obj["right_camera_to_world"] = Serialization::serializePose(right_camera_to_world);

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

bool StereoRigCalibrationData::loadFromFile(const std::string& path)
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
            left_camera_to_world = Serialization::deserializePose(root["left_camera_to_world"]);
            right_camera_to_world = Serialization::deserializePose(root["right_camera_to_world"]);
        }
        catch(SerializationError& err)
        {
            ok = false;
        }
    }

    return ok;
}

