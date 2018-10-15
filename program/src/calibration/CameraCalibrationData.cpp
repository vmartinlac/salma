#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <fstream>
#include <iostream>
#include "CameraCalibrationData.h"

CameraCalibrationData::CameraCalibrationData()
{
}

bool CameraCalibrationData::saveToFile(const std::string& path)
{
    auto mat2json = [] (const cv::Mat& from) -> QJsonArray
    {
        QJsonArray to;

        for(int i=0; i<3; i++)
        {
            for(int j=0; j<3; j++)
            {
                to.push_back(from.at<double>(i,j));
            }
        }

        return to;
    };

    auto size2json = [] (const cv::Size& from) -> QJsonArray
    {
        QJsonArray to;

        to.push_back(from.width);
        to.push_back(from.height);

        return to;
    };

    QJsonObject obj;
    obj["calibration_matrix"] = mat2json(calibration_matrix);
    obj["distortion_coefficients"] = mat2json(distortion_coefficients);
    obj["image_size"] = size2json(image_size);

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

bool CameraCalibrationData::loadFromFile(const std::string& path)
{
    throw std::runtime_error("Not implemented!"); // TODO
}

