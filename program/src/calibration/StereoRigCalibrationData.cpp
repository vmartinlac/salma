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
    throw std::runtime_error("Not implemented!"); // TODO
}

bool StereoRigCalibrationData::loadFromFile(const std::string& path)
{
    throw std::runtime_error("Not implemented!"); // TODO
}

