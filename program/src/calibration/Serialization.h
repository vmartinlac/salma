#pragma once

#include <QJsonValue>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>
#include <stdexcept>

class SerializationError : public std::runtime_error
{
public:
    SerializationError();
};

class Serialization
{
public:

    static QJsonValue serializePose(const Sophus::SE3d& from);
    static Sophus::SE3d deserializePose(const QJsonValue& from);

    static QJsonValue serializeCalibrationMatrix(const cv::Mat& from);
    static cv::Mat deserializeCalibrationMatrix(const QJsonValue& from);

    static QJsonValue serializeDistortionCoefficients(const cv::Mat& from);
    static cv::Mat deserializeDistortionCoefficients(const QJsonValue& from);

    static QJsonValue serializeSize(const cv::Size& size);
    static cv::Size deserializeSize(const QJsonValue& size);

protected:
};
