#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <fstream>
#include <iostream>
#include "StereoRigCalibrationData.h"

StereoRigCalibrationData::StereoRigCalibrationData()
{
}

bool StereoRigCalibrationData::saveToFile(const std::string& path)
{
  throw std::runtime_error("Not implemented!"); //TODO

    QJsonObject obj;
    obj["somevariable"] = "somevalue";

    QJsonDocument doc;
    doc.setObject(obj);

    QByteArray json = doc.toJson();

    std::cout << json.data() << std::endl;

    std::ofstream file(path.c_str());

    if(file.is_open())
    {
        file << json.data() << std::endl;
        file.close();
    }
    else
    {
        return false;
    }

    return true;
}

bool StereoRigCalibrationData::loadFromFile(const std::string& path)
{
    throw std::runtime_error("Not implemented!"); // TODO
}

